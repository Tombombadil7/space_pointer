/*
 * PROJECT: Universal Space Pointer v7.0
 * FEATURES: 
 * - Auto IP-Geolocation & Manual Phone GPS Sync
 * - Real-time Satellite (Sgp4) & Planetary Tracking (Astronomy Lib)
 * - Live Leaflet.js Map Interface
 * - Custom NORAD ID tracking
 * - IMU BNO055 Compensation (Yaw & Pitch) 
 */

#include <WiFiManager.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tracking_math.h"
#include "web_ui.h" // Browser dashboard markup, styles, and JavaScript
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <AS5600.h> // I2C magnetic encoder for elevation feedback
// (install: https://github.com/RobTillaart/AS5600)
#include <Adafruit_BNO055.h>
#include <Sgp4.h>
#include <astronomy.h>

// --- all global structures and definitions ---
// ── Pin definitions ────────────────────────────────────────────────────
// Azimuth A4988
#define AZ_STEP_PIN    12
#define AZ_DIR_PIN     13
#define AZ_EN_PIN      16    // LOW = enabled; GPIO16 on ESP32-S3-DevKitC-1

// Elevation A4988
#define EL_STEP_PIN    14
#define EL_DIR_PIN     15
#define EL_EN_PIN      17    // LOW = enabled; GPIO17 on ESP32-S3-DevKitC-1
// MS pins: tie physically to 3.3V for 1/16 microstepping (no code needed)
// A4988: MS1=HIGH, MS2=HIGH, MS3=HIGH → 1/16 step

// ── Motor constants ─────────────────────────────────────────────────────
// Set this for the actual bipolar motor, gearbox, and microstep configuration.
#define MICROSTEPS_PER_REV  32768UL
#define STEP_PULSE_US     50      // µs per pulse
#define MOTOR_TASK_MS     25      // MotorTask period in ms
// Absolute per-tick limit for an unexpected large correction.
// Normal tracking corrections should be much smaller; this is not an acceleration ramp.
#define MAX_STEPS_PER_TICK  8

// Magnetic declination at the physical installation site, in degrees.
// East is positive; west is negative. Set this using a current magnetic
// declination model for the telescope's location. The BNO055 reports
// magnetic-north heading, while Astronomy Engine calculates true-north azimuth.
// A zero value is only correct where magnetic and true north coincide.
#define MAGNETIC_DECLINATION_DEG  0.0f

// AS5600 elevation calibration.
// Set ENCODER_ZERO_RAW to the raw sensor reading when the telescope tube is
// physically level (0 degrees elevation). The default 2048 preserves the
// previous assumption that the sensor's half-turn position is level.
// Set ENCODER_DIRECTION to +1 or -1 after checking which way the raw reading
// changes when the telescope is moved upward. Do not use AS5600.setDirection()
// as well: direction is applied explicitly below.
#define ENCODER_ZERO_RAW       2048
#define ENCODER_DIRECTION      1

AsyncWebServer server(80);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Sgp4 satSgp4;

struct {
  // observer location
  double lat, lon, alt;

  // pointing targets
  double targetAz, targetEl;

  // satellite map position
  double satLat, satLon;

  // telemetry data exposed through the web API
  double satSpeed;       // km/s
  double satAltKm;       // km above Earth surface (satellites) or from Earth center (planets)
  double satDist;        // km from observer

  // next pass / rise+set
  bool   nextPassValid  = false;
  time_t nextPassAOS    = 0;   // unix timestamp — rise time or satellite AOS
  time_t nextPassLOS    = 0;   // unix timestamp — set time or satellite LOS
  double nextPassMaxEl  = 0;   // degrees — satellite only, 0 for planets

} sys;

enum TargetType {
  TYPE_SATELLITE = 0,
  TYPE_PLANET    = 1
};

struct TargetState {
  TargetType type;
  union { long noradID; int body; };
  char name[25];
};
TargetState current = {TYPE_SATELLITE, {.noradID = 25544}, "ISS"};
// Incremented whenever the selected target changes. PhysicsTask and MotorTask
// use this version to reject calculations/commands produced for an older target.
// Access is protected by targetMutex, just like current and motorTarget.
uint32_t targetGeneration = 1;

// Incremented whenever the observer's location changes. Protected by sysMutex,
// alongside sys.lat/lon/alt, so a calculation can verify its location snapshot.
uint32_t observerGeneration = 1;

struct VersionedTargetSnapshot {
  TargetState target;
  uint32_t generation;
};
// A deliberately simple proportional controller is enough for slow telescope tracking.
// It avoids derivative kick and integral wind-up when a new target is selected.
struct PController {
  float kp;
  float deadband;
};
struct MotorTarget { float az = 0; float el = 0; } motorTarget;

// Modest gains suit the small, repeated corrections used during normal tracking.
// A narrow deadband filters sensor noise without allowing a large pointing error.
PController pAz = {2.5f, 0.05f};
PController pEl = {3.0f, 0.05f};
SemaphoreHandle_t targetMutex;   // protects current, motorTarget, and calculationReadiness
SemaphoreHandle_t i2cMutex;      // protects the shared I2C bus
SemaphoreHandle_t sysMutex;      // protects observer and telemetry state
SemaphoreHandle_t sgp4Mutex;
// The observer/target pair used to initialize satSgp4; protected by sgp4Mutex.
TrackingMath::GenerationReadiness tleSourceVersions;     // protects the shared SGP4 instance
SemaphoreHandle_t tleFetchMutex;
// One long-lived worker handles all TLE refresh requests. Notifications are
// coalesced while a download is running, so rapid target changes cannot create
// an unbounded number of short-lived FreeRTOS tasks.
TaskHandle_t tleFetchTaskHandle = NULL;

volatile bool bnoReady = false;
volatile bool encoderReady = false;
volatile bool observerReady = false;
volatile bool timeReady = false;
volatile bool tleReady = false;
// Protected by targetMutex. Readiness is valid only for this exact target/location pair.
TrackingMath::GenerationReadiness calculationReadiness;

// Other tasks may request a safe stop, but only MotorTask writes motor GPIOs.
// MotorTask consumes this flag and performs the actual ENABLE-pin changes.
volatile bool motorStopRequested = true;

// Change both credentials before deploying outside a trusted test network.
static const char *CONTROL_USER = "admin";
static const char *CONTROL_PASSWORD = "spacepointer-change-me";
static const char *SETUP_AP_PASSWORD = "SpacePointerSetup";

struct TelemetrySnapshot {
  double targetAz, targetEl;
  double satLat, satLon;
  double satSpeed, satAltKm, satDist;
  bool nextPassValid;
  time_t nextPassAOS, nextPassLOS;
  double nextPassMaxEl;
};
// ── AS5600 encoder ──────────────────────────────────────────────────────
AS5600 encoder;   // default I2C address 0x36, shares bus with BNO055 (0x28)
bool updateIPLocation();
void fetchTLE();
bool startTLEFetchTask();
void TLEFetchTask(void * p);
TargetState getTargetSnapshot();
VersionedTargetSnapshot getVersionedTargetSnapshot();
bool getTelemetrySnapshot(TelemetrySnapshot &snapshot);
bool getVersionedObserverSnapshot(double &lat, double &lon, double &alt,
                                  uint32_t &generation);
bool parseDoubleArg(AsyncWebServerRequest *request, const char *name, double &value);
void disableMotors();
// Clear readiness after releasing sysMutex; never acquire targetMutex from sysMutex.
void invalidateCalculationReadiness();
const char *planetName(int body);
void PhysicsTask(void * p);
void updateNextPass(const TargetState &target, uint32_t targetVersion,
                    uint32_t observerVersion, time_t now,
                    double lat, double lon, double alt);
void stepMotor(int stepPin, int dirPin, bool forward);
int pidOutput(const PController &p, float error);
void MotorTask(void * p);





void setup() {
  // ── 1. System Initialization ───────────────────────────────────────
  Serial.begin(115200);
  Serial.println("\nStarting Universal Space Pointer...");

  // ── 2. FreeRTOS Primitives ─────────────────────────────────────────
  targetMutex   = xSemaphoreCreateMutex();
  i2cMutex      = xSemaphoreCreateMutex();
  sysMutex      = xSemaphoreCreateMutex();
  sgp4Mutex     = xSemaphoreCreateMutex();
  tleFetchMutex = xSemaphoreCreateMutex();
  if (targetMutex == NULL || i2cMutex == NULL || sysMutex == NULL ||
      sgp4Mutex == NULL || tleFetchMutex == NULL) {
      Serial.println("CRITICAL: Failed to create mutexes! Halting.");
      while (1) { delay(1000); }
  }

  // Create the TLE worker once during startup. HTTP handlers only notify this
  // task; they never allocate a new task for each target selection.
  if (xTaskCreate(TLEFetchTask, "TLEFetch", 10000, NULL, 1,
                  &tleFetchTaskHandle) != pdPASS) {
      tleFetchTaskHandle = NULL;
      Serial.println("WARNING: Could not create TLE worker; satellite refreshes will be unavailable.");
  }

  // ── 3. Hardware Initialization ─────────────────────────────────────
  pinMode(AZ_STEP_PIN, OUTPUT); pinMode(AZ_DIR_PIN, OUTPUT);
  pinMode(EL_STEP_PIN, OUTPUT); pinMode(EL_DIR_PIN, OUTPUT);
  // Set a safe initial electrical state before the FreeRTOS motor task starts.
  // After startup, MotorTask is the only task allowed to write motor GPIOs.
  pinMode(AZ_EN_PIN, OUTPUT);   digitalWrite(AZ_EN_PIN, HIGH); // disabled
  pinMode(EL_EN_PIN, OUTPUT);   digitalWrite(EL_EN_PIN, HIGH); // disabled
  digitalWrite(AZ_STEP_PIN, LOW);
  digitalWrite(EL_STEP_PIN, LOW);

  // ESP32-S3 DevKitC-1 I2C mapping; all three peripherals share this bus.
  Wire.begin(8, 9); // SDA=GPIO8, SCL=GPIO9
  Wire.setClock(400000);

  // ── AS5600 encoder ─────────────────────────────────────────────────
  encoder.begin();
  encoderReady = encoder.isConnected();
  if (!encoderReady) {
      Serial.println("WARNING: AS5600 not found — motor control will stay disabled");
  }

  // ── BNO055 IMU ─────────────────────────────────────────────────────
  bnoReady = bno.begin();
  if (!bnoReady) {
      Serial.println("WARNING: BNO055 not found — motor control will stay disabled");
  }

  // ── 4. Network & Web Services ──────────────────────────────────────
  WiFiManager wm;
  if (!wm.autoConnect("SpaceTracker_AP", SETUP_AP_PASSWORD)) {
      Serial.println("ERROR: Wi-Fi connection/configuration failed; motors remain disabled.");
      disableMotors();
      while (1) { delay(1000); }
  }
  observerReady = updateIPLocation();
  if (!observerReady) {
      Serial.println("WARNING: IP geolocation failed; use Sync Phone GPS before tracking.");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
      if (!r->authenticate(CONTROL_USER, CONTROL_PASSWORD)) {
          r->requestAuthentication();
          return;
      }
      r->send(200, "text/html", index_html);
  });

  server.on("/updateLoc", HTTP_GET, [](AsyncWebServerRequest *r){
      if (!r->authenticate(CONTROL_USER, CONTROL_PASSWORD)) {
          r->requestAuthentication();
          return;
      }
      double lat, lon, alt;
      if (!parseDoubleArg(r, "lat", lat) || !parseDoubleArg(r, "lon", lon) ||
          !parseDoubleArg(r, "alt", alt) ||
          !TrackingMath::validObserver(lat, lon, alt)) {
          r->send(400, "text/plain", "Invalid location. Check latitude, longitude and altitude.");
          return;
      }
      if (xSemaphoreTake(sysMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
          r->send(503, "text/plain", "System busy; retry location update.");
          return;
      }
      sys.lat = lat; sys.lon = lon; sys.alt = alt;
      ++observerGeneration;
      if (observerGeneration == 0) ++observerGeneration; // Reserve zero for an invalid snapshot.
      xSemaphoreGive(sysMutex);
      observerReady = true;
      invalidateCalculationReadiness();

      TargetState target = getTargetSnapshot();
      if (target.type == TYPE_SATELLITE) {
          tleReady = false;   // invalidate the old observer's SGP4 state immediately
          if (!startTLEFetchTask()) {
              r->send(503, "text/plain", "Could not start TLE refresh; motors remain disabled.");
              return;
          }
      }
      r->send(202, "text/plain", "Location accepted.");
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *r){
      if (!r->authenticate(CONTROL_USER, CONTROL_PASSWORD)) {
          r->requestAuthentication();
          return;
      }
      if (!r->hasArg("type") || !r->hasArg("val")) {
          r->send(400, "text/plain", "Missing target type or value.");
          return;
      }
      char *end = nullptr;
      const String typeArg = r->arg("type");
      const String valArg = r->arg("val");
      long type = strtol(typeArg.c_str(), &end, 10);
      if (end == typeArg.c_str() || *end != '\0' || (type != 0 && type != 1)) {
          r->send(400, "text/plain", "Invalid target type.");
          return;
      }
      end = nullptr;
      long val = strtol(valArg.c_str(), &end, 10);
      if (end == valArg.c_str() || *end != '\0') {
          r->send(400, "text/plain", "Invalid target ID.");
          return;
      }

      TargetState next = {};
      if (type == TYPE_SATELLITE) {
          if (!TrackingMath::validNoradId(val)) {
              r->send(400, "text/plain", "NORAD ID is outside the accepted range.");
              return;
          }
          next.type = TYPE_SATELLITE;
          next.noradID = val;
          String requestedName = r->arg("name");
          requestedName.trim();
          if (requestedName.length() > 0) {
              requestedName.toCharArray(next.name, sizeof(next.name));
              next.name[sizeof(next.name) - 1] = 0;
          } else {
              snprintf(next.name, sizeof(next.name), "NORAD %ld", val);
          }
      } else {
          if (!TrackingMath::supportedPlanetBody((int)val)) {
              r->send(400, "text/plain", "Unsupported planetary body.");
              return;
          }
          next.type = TYPE_PLANET;
          next.body = (int)val;
          strlcpy(next.name, planetName((int)val), sizeof(next.name));
      }

      if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
          r->send(503, "text/plain", "Target controller busy; retry.");
          return;
      }
      if (next.type == TYPE_SATELLITE) {
          tleReady = false;   // never track a newly selected NORAD ID with the previous TLE
      }
      calculationReadiness.clear(); // targetMutex is already held here.
      current = next;
      ++targetGeneration;
      if (targetGeneration == 0) ++targetGeneration; // Reserve zero for an invalid snapshot.
      motorTarget.az = 0;
      motorTarget.el = 0;
      xSemaphoreGive(targetMutex);

      if (next.type == TYPE_SATELLITE) {
          if (!startTLEFetchTask()) {
              r->send(503, "text/plain", "Could not start TLE download; motors remain disabled.");
              return;
          }
      }
      r->send(202, "text/plain", "Target accepted.");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *r){
      const TargetState target = getTargetSnapshot();
      TelemetrySnapshot data = {};
      // Never serialize a zero-filled snapshot when the telemetry lock is busy.
      // The dashboard can then show a local telemetry error and keep the rest
      // of the page (map and target controls) fully available.
      if (!getTelemetrySnapshot(data)) {
          r->send(503, "text/plain", "Telemetry temporarily unavailable; please retry.");
          return;
      }
      StaticJsonDocument<512> doc;
      doc["name"]  = target.name;
      doc["type"]  = target.type;
      doc["az"]    = data.targetAz;
      doc["el"]    = data.targetEl;
      doc["sLat"]  = data.satLat;
      doc["sLon"]  = data.satLon;
      doc["speed"] = data.satSpeed;
      doc["alt"]   = data.satAltKm;
      doc["dist"]  = data.satDist;
      if (data.nextPassValid) {
          char aos[24], los[24];
          struct tm aosUtc, losUtc;
          gmtime_r(&data.nextPassAOS, &aosUtc);
          gmtime_r(&data.nextPassLOS, &losUtc);
          snprintf(aos, sizeof(aos), "%02d:%02d UTC", aosUtc.tm_hour, aosUtc.tm_min);
          snprintf(los, sizeof(los), "%02d:%02d UTC", losUtc.tm_hour, losUtc.tm_min);
          doc["nextPass"] = aos;
          doc["nextPassLOS"] = los;
          doc["nextPassEl"] = (int)data.nextPassMaxEl;
      } else {
          doc["nextPass"] = "--:--";
          doc["nextPassLOS"] = "--:--";
          doc["nextPassEl"] = 0;
      }
      String out;
      serializeJson(doc, out);
      r->send(200, "application/json", out);
  });

  server.begin();

  // ── 5. Time Synchronization ────────────────────────────────────────
  // Astronomy calculations are based on UTC; do not apply local timezone offsets.
  configTime(0, 0, "pool.ntp.org");
  Serial.print("Waiting for NTP time sync...");
  time_t now = 0;
  const uint32_t ntpStart = millis();
  while (time(&now) < 1000000000 && millis() - ntpStart < 45000) {
      delay(250);
      Serial.print(".");
  }
  timeReady = (time(&now) >= 1000000000);
  if (timeReady) {
      Serial.println("\nTime synchronized (UTC)!");
  } else {
      Serial.println("\nWARNING: NTP sync timed out; motor tracking remains disabled.");
  }

  // Load the default ISS TLE before the physics task can use the SGP4 object.
  if (timeReady && observerReady) {
      fetchTLE();
  }

  // ── 6. Task Creation ───────────────────────────────────────────────
  // WiFi stack runs on Core 0 — put Physics on Core 1 to avoid contention
  if (xTaskCreatePinnedToCore(PhysicsTask, "Physics", 15000, NULL, 2, NULL, 1) != pdPASS) {
      Serial.println("CRITICAL: Physics task creation failed.");
      disableMotors();
  }
  if (xTaskCreatePinnedToCore(MotorTask, "Motors", 8000, NULL, 3, NULL, 0) != pdPASS) {
      Serial.println("CRITICAL: Motor task creation failed.");
      disableMotors();
  }

}

void loop() {
  vTaskDelay(portMAX_DELAY);
}



void invalidateCalculationReadiness() {
  // Call only when sysMutex is not held. Nested locking always follows
  // targetMutex -> sysMutex, preventing a reverse-order deadlock.
  if (targetMutex != NULL &&
      xSemaphoreTake(targetMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      calculationReadiness.clear();
      xSemaphoreGive(targetMutex);
  }
}

VersionedTargetSnapshot getVersionedTargetSnapshot() {
  VersionedTargetSnapshot result = {};
  if (targetMutex != NULL && xSemaphoreTake(targetMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      result.target = current;
      result.generation = targetGeneration;
      xSemaphoreGive(targetMutex);
  }
  // A zero generation means the mutex could not be acquired; callers must
  // treat that snapshot as unavailable instead of calculating with defaults.
  return result;
}

TargetState getTargetSnapshot() {
  return getVersionedTargetSnapshot().target;
}

bool getTelemetrySnapshot(TelemetrySnapshot &snapshot) {
  // A failed lock is not valid telemetry. Return false so the HTTP handler
  // can report a temporary service error instead of publishing fake zeros.
  if (sysMutex == NULL || xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
      return false;
  }

  snapshot.targetAz = sys.targetAz; snapshot.targetEl = sys.targetEl;
  snapshot.satLat = sys.satLat; snapshot.satLon = sys.satLon;
  snapshot.satSpeed = sys.satSpeed; snapshot.satAltKm = sys.satAltKm;
  snapshot.satDist = sys.satDist;
  snapshot.nextPassValid = sys.nextPassValid;
  snapshot.nextPassAOS = sys.nextPassAOS; snapshot.nextPassLOS = sys.nextPassLOS;
  snapshot.nextPassMaxEl = sys.nextPassMaxEl;
  xSemaphoreGive(sysMutex);
  return true;
}

bool getVersionedObserverSnapshot(double &lat, double &lon, double &alt,
                                  uint32_t &generation) {
  // Read the coordinates and their version under the same lock. This ensures
  // the version always describes the exact location copied into this snapshot.
  if (sysMutex == NULL || xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
      return false;
  }

  lat = sys.lat;
  lon = sys.lon;
  alt = sys.alt;
  generation = observerGeneration;
  xSemaphoreGive(sysMutex);

  return generation != 0 && TrackingMath::validObserver(lat, lon, alt);
}

bool parseDoubleArg(AsyncWebServerRequest *request, const char *name, double &value) {
  if (!request->hasArg(name)) return false;
  const String arg = request->arg(name);
  char *end = nullptr;
  value = strtod(arg.c_str(), &end);
  return end != arg.c_str() && *end == '\0' && isfinite(value);
}

const char *planetName(int body) {
  switch (body) {
    case BODY_MOON: return "Moon";
    case BODY_MARS: return "Mars";
    case BODY_JUPITER: return "Jupiter";
    case BODY_SATURN: return "Saturn";
    default: return nullptr;
  }
}

// Request a stop from any task. This function deliberately does not touch GPIO:
// MotorTask owns all motor outputs and will disable the drivers on its next tick.
void disableMotors() {
  motorStopRequested = true;
}

// Ask the persistent worker to refresh TLE data. FreeRTOS task notifications
// are counted, but the worker clears pending notifications when it wakes; this
// intentionally coalesces a burst of requests into one refresh of the latest target.
bool startTLEFetchTask() {
  tleReady = false;
  invalidateCalculationReadiness();
  if (tleFetchTaskHandle == NULL) return false;
  xTaskNotifyGive(tleFetchTaskHandle);
  return true;
}

// The worker sleeps without consuming CPU until a refresh is requested. If
// several requests arrive during a download, the next wake handles the latest
// selected target rather than spawning additional tasks.
void TLEFetchTask(void * p) {
  for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      fetchTLE();
  }
}

int pidOutput(const PController &p, float error) {
    // Keep the motor controller thin: the math is isolated in a portable,
    // hardware-independent helper so its edge cases can be unit-tested.
    return TrackingMath::proportionalSteps(
        p.kp, p.deadband, error, MAX_STEPS_PER_TICK);
}

// Calculate the next satellite pass or planetary rise/set independently of
// any display. The web API reads the result from the shared telemetry state.
void updateNextPass(const TargetState &target, uint32_t targetVersion,
                    uint32_t observerVersion, time_t now,
                    double lat, double lon, double alt) {
    bool passValid = false;
    time_t aosTime = 0;
    time_t losTime = 0;
    double maxElevation = 0.0;

    if (target.type == TYPE_SATELLITE && tleReady &&
        xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Never calculate a pass with SGP4 data belonging to an older target
        // or observer location. The source token is protected by this mutex.
        if (tleSourceVersions.matches(targetVersion, observerVersion)) {
            // Search for a pass that reaches at least 20 degrees above the horizon.
            // The second argument is the search iteration limit, not the elevation.
            const bool predictionReady =
                satSgp4.initpredpoint((unsigned long)now, 0.0);
            passinfo overpass = {};
            passValid = predictionReady &&
                        satSgp4.nextpass(&overpass, 20, false, 20.0);
            if (passValid) {
                aosTime = (time_t)((overpass.jdstart - 2440587.5) * 86400.0);
                losTime = (time_t)((overpass.jdstop  - 2440587.5) * 86400.0);
                maxElevation = overpass.maxelevation;
            }
        }
        xSemaphoreGive(sgp4Mutex);
    } else if (target.type == TYPE_PLANET) {
        struct tm utc;
        if (gmtime_r(&now, &utc) != nullptr) {
            astro_time_t start = Astronomy_MakeTime(
                utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                utc.tm_hour, utc.tm_min, (double)utc.tm_sec);
            const astro_observer_t observer = {lat, lon, alt};
            const astro_search_result_t rise = Astronomy_SearchRiseSet(
                (astro_body_t)target.body, observer, DIRECTION_RISE, start, 1.0);
            const astro_search_result_t set = Astronomy_SearchRiseSet(
                (astro_body_t)target.body, observer, DIRECTION_SET, start, 1.0);
            if (rise.status == ASTRO_SUCCESS && set.status == ASTRO_SUCCESS) {
                aosTime = (time_t)(rise.time.ut * 86400.0 + 946728000.0);
                losTime = (time_t)(set.time.ut * 86400.0 + 946728000.0);
                passValid = true;
            }
        }
    }

    // A pass prediction can take noticeable time. Publish it only if neither
    // the selected target nor the observer location changed during the search.
    // Lock order stays targetMutex -> sysMutex, matching the physics commit path.
    if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (targetGeneration == targetVersion &&
            xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (observerGeneration == observerVersion) {
                sys.nextPassValid = passValid;
                sys.nextPassAOS = aosTime;
                sys.nextPassLOS = losTime;
                sys.nextPassMaxEl = maxElevation;
            }
            xSemaphoreGive(sysMutex);
        }
        xSemaphoreGive(targetMutex);
    }
}

void PhysicsTask(void * p) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
      if (!timeReady) {
          time_t syncedTime = 0;
          time(&syncedTime);
          if (syncedTime >= 1000000000) {
              timeReady = true;   // NTP may have synchronized after the startup timeout.
              Serial.println("NTP time became available; tracking can resume.");
          }
      }
      if (!timeReady || !observerReady) {
          // MotorTask checks these prerequisites independently. A delayed
          // physics iteration must not erase a newer successful token.
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }

      const VersionedTargetSnapshot targetSnapshot = getVersionedTargetSnapshot();
      if (targetSnapshot.generation == 0) {
          // No consistent target snapshot was available; do not publish a token.
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }
      const TargetState target = targetSnapshot.target;
      const uint32_t calculationGeneration = targetSnapshot.generation;
      double lat, lon, alt;
      uint32_t calculationObserverGeneration = 0;
      if (!getVersionedObserverSnapshot(lat, lon, alt,
                                        calculationObserverGeneration)) {
          // No observer snapshot was available; the motor task will fail safe.
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }

      time_t now;
      time(&now);
      if (now < 1000000000) {
          // MotorTask independently checks timeReady before enabling the drivers.
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }

      // Pass/rise/set predictions are telemetry too, so update them here
      // rather than tying their calculation to a physical display task.
      static uint32_t lastPassCalc = 0;
      if (millis() - lastPassCalc > 30000) {
          updateNextPass(target, calculationGeneration,
                         calculationObserverGeneration, now, lat, lon, alt);
          lastPassCalc = millis();
      }

      double targetAz = 0.0, targetEl = 0.0;
      double satLat = 0.0, satLon = 0.0;
      double satSpeed = 0.0, satAltKm = 0.0, satDist = 0.0;
      bool calculationOK = false;

      if (target.type == TYPE_SATELLITE) {
          if (tleReady && xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
              if (tleSourceVersions.matches(calculationGeneration,
                                            calculationObserverGeneration)) {
                  satSgp4.findsat((unsigned long)now);
                  const double lat1 = satSgp4.satLat;
                  const double lon1 = satSgp4.satLon;
                  const double alt1 = satSgp4.satAlt;
                  const double dist1 = satSgp4.satDist;
                  targetAz = satSgp4.satAz;
                  targetEl = satSgp4.satEl;

                  satSgp4.findsat((unsigned long)(now + 1));
                  const double dlat = (satSgp4.satLat - lat1) * DEG_TO_RAD * 6371.0;
                  const double dlon = TrackingMath::wrapAngle((float)(satSgp4.satLon - lon1)) *
                                      DEG_TO_RAD * 6371.0 * cos(lat1 * DEG_TO_RAD);
                  const double dalt = satSgp4.satAlt - alt1;
                  satSpeed = sqrt(dlat*dlat + dlon*dlon + dalt*dalt);

                  satSgp4.findsat((unsigned long)now);
                  satLat = lat1;
                  satLon = lon1;
                  satAltKm = alt1;
                  satDist = dist1;
                  calculationOK = isfinite(targetAz) && isfinite(targetEl) &&
                                  isfinite(satSpeed) && isfinite(satAltKm) &&
                                  isfinite(satDist);
              } // End exact TLE source-generation match.
              xSemaphoreGive(sgp4Mutex);
          }
      } else {
          struct tm utc;
          if (gmtime_r(&now, &utc) != nullptr) {
              astro_time_t aTime = Astronomy_MakeTime(
                  utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                  utc.tm_hour, utc.tm_min, (double)utc.tm_sec);
              const astro_observer_t obs = {lat, lon, alt};
              astro_equatorial_t equ1 = Astronomy_Equator(
                  (astro_body_t)target.body, &aTime, obs,
                  EQUATOR_OF_DATE, ABERRATION);
              if (equ1.status == ASTRO_SUCCESS) {
                  astro_horizon_t hor1 = Astronomy_Horizon(
                      &aTime, obs, equ1.ra, equ1.dec, REFRACTION_NORMAL);

                  astro_time_t aTime2 = Astronomy_AddDays(aTime, 1.0 / 86400.0);
                  astro_equatorial_t equ2 = Astronomy_Equator(
                      (astro_body_t)target.body, &aTime2, obs,
                      EQUATOR_OF_DATE, ABERRATION);
                  if (equ2.status == ASTRO_SUCCESS) {
                      astro_horizon_t hor2 = Astronomy_Horizon(
                          &aTime2, obs, equ2.ra, equ2.dec, REFRACTION_NORMAL);

                      const double distanceKm = equ1.dist * 149597870.7;
                      const double dDistance = (equ2.dist - equ1.dist) * 149597870.7;
                      const double dAz = TrackingMath::wrapAngle((float)(hor2.azimuth - hor1.azimuth)) *
                                         DEG_TO_RAD * distanceKm;
                      const double dEl = (hor2.altitude - hor1.altitude) *
                                         DEG_TO_RAD * distanceKm;

                      targetAz = hor1.azimuth;
                      targetEl = hor1.altitude;
                      satSpeed = sqrt(dAz*dAz + dEl*dEl + dDistance*dDistance);
                      satAltKm = distanceKm;
                      satDist = distanceKm;
                      calculationOK = isfinite(targetAz) && isfinite(targetEl) &&
                                      isfinite(satSpeed) && isfinite(distanceKm);
                  }
              }

          }
      }

      bool resultCommitted = false;
      if (calculationOK &&
          xSemaphoreTake(targetMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          // Keep targetMutex while committing both telemetry and motor target.
          // A target change cannot slip between the generation check and write.
          if (targetGeneration == calculationGeneration) {
              if (xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                  // Location updates also take sysMutex, so this check and the
                  // telemetry write are atomic with respect to /updateLoc and
                  // the automatic IP-geolocation update.
                  if (observerGeneration == calculationObserverGeneration) {
                      sys.targetAz = targetAz;
                      sys.targetEl = targetEl;
                      sys.satLat = satLat;
                      sys.satLon = satLon;
                      sys.satSpeed = satSpeed;
                      sys.satAltKm = satAltKm;
                      sys.satDist = satDist;
                      resultCommitted = true;
                      // Publish inside the same critical section as telemetry
                      // and motorTarget, while both source generations are locked.
                      calculationReadiness.publish(calculationGeneration,
                                                   calculationObserverGeneration);
                  }
                  xSemaphoreGive(sysMutex);

                  // Publish the motor target only after both versions have
                  // been validated while their protecting locks are held.
                  if (resultCommitted) {
                      motorTarget.az = (float)targetAz;
                      motorTarget.el = (float)targetEl;
                  }
              }
          }
          xSemaphoreGive(targetMutex);
      }
      // Failed/stale calculations do not clear readiness: an older failed
      // iteration must never erase a newer successful generation pair.

      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
  }
}




void MotorTask(void * p) {
  TickType_t lastWake = xTaskGetTickCount();
  bool driversEnabled = false;

  for (;;) {
      float targetAz = 0.0f, targetEl = 0.0f;
      TargetState activeTarget = {};
      bool targetRead = false;
      bool calculationReady = false;
      uint32_t activeGeneration = 0;
      if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          activeTarget = current;
          activeGeneration = targetGeneration;
          targetAz = motorTarget.az;
          targetEl = motorTarget.el;

          // Read the live observer generation under the global lock order:
          // targetMutex first, then sysMutex.
          if (xSemaphoreTake(sysMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
              calculationReady = calculationReadiness.matches(
                  targetGeneration, observerGeneration);
              targetRead = true;
              xSemaphoreGive(sysMutex);
          }
          xSemaphoreGive(targetMutex);
      }

      const bool orbitalDataReady = activeTarget.type != TYPE_SATELLITE || tleReady;
      if (!targetRead || !timeReady || !observerReady || !calculationReady ||
          !orbitalDataReady || !bnoReady || !encoderReady || !isfinite(targetAz) ||
          !isfinite(targetEl) || targetEl < 0.0f || targetEl > 90.0f) {
          // Only this task changes the driver ENABLE pins during operation.
          if (driversEnabled) {
              digitalWrite(AZ_EN_PIN, HIGH);
              digitalWrite(EL_EN_PIN, HIGH);
              driversEnabled = false;
          }
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }

      // Consume stop requests raised by networking, TLE, or other tasks.
      // The request forces at least one complete motor-control tick to remain stopped.
      if (motorStopRequested) {
          motorStopRequested = false;
          if (driversEnabled) {
              digitalWrite(AZ_EN_PIN, HIGH);
              digitalWrite(EL_EN_PIN, HIGH);
              driversEnabled = false;
          }
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }

      sensors_event_t event = {};
      uint16_t encoderRaw = 0;
      bool sensorsRead = false;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          const bool headingRead = bno.getEvent(&event);
          uint8_t systemCal = 0, gyroCal = 0, accelCal = 0, magCal = 0;
          bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
          encoderRaw = encoder.rawAngle();
          // Magnetic heading must be fully calibrated before pointing from true-north targets.
          sensorsRead = headingRead && systemCal == 3 && magCal == 3 &&
                        (encoder.lastError() == AS5600_OK);
          xSemaphoreGive(i2cMutex);
      }

      const float headingNow = event.orientation.x;
      if (!sensorsRead || !isfinite(headingNow) || headingNow < 0.0f ||
          headingNow >= 360.0f || encoderRaw > 4095) {
          if (driversEnabled) {
              digitalWrite(AZ_EN_PIN, HIGH);
              digitalWrite(EL_EN_PIN, HIGH);
              driversEnabled = false;
          }
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }
      // Convert the 12-bit encoder reading into elevation relative to the
      // calibrated level position. Wrap the difference so small movements
      // across the sensor's 0/4095 boundary remain continuous.
      int32_t encoderDelta = (int32_t)encoderRaw - ENCODER_ZERO_RAW;
      if (encoderDelta > 2048) encoderDelta -= 4096;
      if (encoderDelta < -2048) encoderDelta += 4096;

      const float elNow = ENCODER_DIRECTION *
                          encoderDelta * (360.0f / 4096.0f);

      // Convert the BNO055 magnetic heading to true north before comparing
      // it with Astronomy Engine's true-north target azimuth.
      float headingTrue = fmodf(headingNow + MAGNETIC_DECLINATION_DEG, 360.0f);
      if (headingTrue < 0.0f) headingTrue += 360.0f;

      const int azSteps = pidOutput(pAz, TrackingMath::wrapAngle(targetAz - headingTrue));
      const int elSteps = pidOutput(pEl, TrackingMath::wrapAngle(targetEl - elNow));
      const bool azFwd = azSteps > 0;
      const bool elFwd = elSteps > 0;
      const int azN = abs(azSteps);
      const int elN = abs(elSteps);
      const int maxN = max(azN, elN);

      // Revalidate immediately before touching the motors. Keep targetMutex
      // for this short, bounded step batch so a new target cannot be selected
      // halfway through executing a command from the previous target.
      if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          // Hold both locks through this short bounded pulse batch. This keeps
          // observerGeneration stable from validation until the motor steps end.
          const bool systemStateLocked =
              xSemaphoreTake(sysMutex, pdMS_TO_TICKS(10)) == pdTRUE;
          const bool commandStillCurrent = systemStateLocked &&
              targetGeneration == activeGeneration &&
              calculationReadiness.matches(targetGeneration, observerGeneration) &&
              !motorStopRequested;
          if (commandStillCurrent) {
              if (!driversEnabled) {
                  digitalWrite(AZ_EN_PIN, LOW);
                  digitalWrite(EL_EN_PIN, LOW);
                  driversEnabled = true;
              }
              for (int i = 0; i < maxN; i++) {
                  if (i < azN) stepMotor(AZ_STEP_PIN, AZ_DIR_PIN, azFwd);
                  if (i < elN) stepMotor(EL_STEP_PIN, EL_DIR_PIN, elFwd);
              }
          } else if (driversEnabled) {
              digitalWrite(AZ_EN_PIN, HIGH);
              digitalWrite(EL_EN_PIN, HIGH);
              driversEnabled = false;
          }
          if (systemStateLocked) xSemaphoreGive(sysMutex);
          xSemaphoreGive(targetMutex);
      } else if (driversEnabled) {
          // If we cannot verify the command's generation, fail safe and stop.
          digitalWrite(AZ_EN_PIN, HIGH);
          digitalWrite(EL_EN_PIN, HIGH);
          driversEnabled = false;
      }
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
  }
}




void stepMotor(int stepPin, int dirPin, bool forward) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  delayMicroseconds(2);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_PULSE_US);
}


// --- לוגיקת שרת וחישובים ---

bool updateIPLocation() {
    HTTPClient http;
    http.setTimeout(8000);
    if (!http.begin("http://ip-api.com/json/")) return false;
    const int status = http.GET();
    bool valid = false;
    if (status == HTTP_CODE_OK) {
        StaticJsonDocument<512> doc;
        const DeserializationError error = deserializeJson(doc, http.getString());
        if (!error && doc["status"] == "success") {
            const double lat = doc["lat"] | NAN;
            const double lon = doc["lon"] | NAN;
            if (isfinite(lat) && isfinite(lon) && lat >= -90.0 &&
                lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
                if (xSemaphoreTake(sysMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    sys.lat = lat;
                    sys.lon = lon;
                    sys.alt = 50.0;
                    ++observerGeneration;
                    if (observerGeneration == 0) ++observerGeneration;
                    xSemaphoreGive(sysMutex);
                    valid = true;
                }
            }
        }
    }
    http.end();
    // Keep this path safe if IP geolocation is ever requested after startup.
    // The observer generation already rejects stale tokens; clearing is explicit.
    if (valid) invalidateCalculationReadiness();
    return valid;
}

void fetchTLE() {
    if (tleFetchMutex == NULL ||
        xSemaphoreTake(tleFetchMutex, pdMS_TO_TICKS(20000)) != pdTRUE) {
        tleReady = false;
        invalidateCalculationReadiness();
        disableMotors();
        return;
    }

    tleReady = false;
    invalidateCalculationReadiness();
    const VersionedTargetSnapshot requestedSnapshot = getVersionedTargetSnapshot();
    const TargetState requested = requestedSnapshot.target;
    double lat, lon, alt;
    uint32_t observerVersion = 0;
    bool success = false;

    if (requestedSnapshot.generation != 0 &&
        requested.type == TYPE_SATELLITE &&
        getVersionedObserverSnapshot(lat, lon, alt, observerVersion) &&
        requested.noradID > 0) {
        HTTPClient http;
        http.setTimeout(12000);
        const String url = "https://tle.ivanstanojevic.me/api/tle/" + String(requested.noradID);
        if (http.begin(url)) {
            const int status = http.GET();
            if (status == HTTP_CODE_OK) {
                StaticJsonDocument<1024> doc;
                const DeserializationError error = deserializeJson(doc, http.getString());
                const char *rawLine1 = doc["line1"].as<const char *>();
                const char *rawLine2 = doc["line2"].as<const char *>();

                if (!error && rawLine1 != nullptr && rawLine2 != nullptr &&
                    strlen(rawLine1) >= 69 && strlen(rawLine2) >= 69 &&
                    rawLine1[0] == '1' && rawLine2[0] == '2') {
                    char line1[71] = {};
                    char line2[71] = {};
                    strlcpy(line1, rawLine1, sizeof(line1));
                    strlcpy(line2, rawLine2, sizeof(line2));

                    // Installation is the critical point: while holding the
                    // target and observer locks, prove that neither input
                    // changed during the network request. This follows the
                    // global lock order targetMutex -> sysMutex -> sgp4Mutex.
                    bool targetLocked =
                        xSemaphoreTake(targetMutex, pdMS_TO_TICKS(100)) == pdTRUE;
                    bool observerLocked = false;
                    bool sgp4Locked = false;
                    if (targetLocked) {
                        observerLocked =
                            xSemaphoreTake(sysMutex, pdMS_TO_TICKS(100)) == pdTRUE;
                    }
                    const bool sameTarget = targetLocked &&
                        targetGeneration == requestedSnapshot.generation &&
                        current.type == TYPE_SATELLITE &&
                        current.noradID == requested.noradID;
                    const bool sameObserver = observerLocked &&
                        observerGeneration == observerVersion;
                    if (sameTarget && sameObserver) {
                        sgp4Locked =
                            xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(500)) == pdTRUE;
                    }

                    if (sameTarget && sameObserver && sgp4Locked) {
                        satSgp4.site(lat, lon, alt);
                        satSgp4.init(requested.name, line1, line2);
                        // Record which exact inputs produced this SGP4 model.
                        // PhysicsTask and updateNextPass reject any other pair.
                        tleSourceVersions.publish(requestedSnapshot.generation,
                                                  observerVersion);
                        tleReady = true;
                        success = true;
                    }

                    if (sgp4Locked) xSemaphoreGive(sgp4Mutex);
                    if (observerLocked) xSemaphoreGive(sysMutex);
                    if (targetLocked) xSemaphoreGive(targetMutex);
                } else {
                    Serial.println("ERROR: TLE response was invalid or incomplete.");
                }
            } else {
                Serial.printf("ERROR: TLE server returned HTTP %d\n", status);
            }
            http.end();
        }
    }

    if (!success) {
        // The request may have become obsolete while the HTTP call was in
        // progress (for example, the user selected a planet or another satellite).
        // Only apply failure handling if both original inputs are still current.
        // Otherwise an old failed request could clear readiness for a newer target.
        bool targetLocked =
            xSemaphoreTake(targetMutex, pdMS_TO_TICKS(50)) == pdTRUE;
        bool observerLocked = false;
        if (targetLocked) {
            observerLocked =
                xSemaphoreTake(sysMutex, pdMS_TO_TICKS(50)) == pdTRUE;
        }

        const bool requestStillCurrent = targetLocked && observerLocked &&
            requestedSnapshot.generation != 0 &&
            targetGeneration == requestedSnapshot.generation &&
            current.type == TYPE_SATELLITE &&
            current.noradID == requested.noradID &&
            observerGeneration == observerVersion;

        if (observerLocked) xSemaphoreGive(sysMutex);
        if (targetLocked) xSemaphoreGive(targetMutex);

        if (requestStillCurrent) {
            // This is a genuine failure for the active satellite/location.
            tleReady = false;
            invalidateCalculationReadiness();
            disableMotors();
            Serial.println("WARNING: TLE update failed; satellite motors remain disabled.");
        } else {
            // A newer request/state owns readiness now. Do not touch it.
            Serial.println("INFO: Discarded stale TLE result after target/location changed.");
        }
    }
    xSemaphoreGive(tleFetchMutex);
}