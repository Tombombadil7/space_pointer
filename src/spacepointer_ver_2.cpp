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
// One generation represents all inputs that affect tracking.
struct ConfigurationSnapshot {
  TargetState target;
  uint32_t generation;
  bool locationReady;
  double lat, lon, alt;
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
SemaphoreHandle_t stateMutex;    // protects target, location, telemetry, and readiness
SemaphoreHandle_t i2cMutex;      // protects the shared I2C bus
SemaphoreHandle_t sgp4Mutex;     // protects the shared SGP4 instance
SemaphoreHandle_t tleFetchMutex; // serializes TLE downloads and installation
// One long-lived worker handles all TLE refresh requests. Notifications are
// coalesced while a download is running, so rapid target changes cannot create
// an unbounded number of short-lived FreeRTOS tasks.
TaskHandle_t tleFetchTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

// These sensors are initialized once during setup.
bool sensorsReady = false;
// Protected by stateMutex; one generation invalidates all derived results.
TrackingMath::ConfigurationReadiness systemReadiness;
static const uint32_t MOTOR_STOP_NOTIFICATION = 1UL << 0;

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
ConfigurationSnapshot getConfigurationSnapshot();
bool getTelemetrySnapshot(TargetState &target, TelemetrySnapshot &snapshot);
bool isTimeReady();
bool lockTleForGeneration(uint32_t generation);
bool parseDoubleArg(AsyncWebServerRequest *request, const char *name, double &value);
// Require a non-simple custom header on state-changing requests to block browser CSRF.
bool hasValidMutationRequest(AsyncWebServerRequest *request);
void disableMotors();
// Call while stateMutex is held after a target or observer change.
void advanceConfigurationLocked();
const char *planetName(int body);
void PhysicsTask(void * p);
void updateNextPass(const ConfigurationSnapshot &snapshot, time_t now);
void stepMotor(int stepPin, int dirPin, bool forward);
int pidOutput(const PController &p, float error);
void MotorTask(void * p);





void setup() {
  // ── 1. System Initialization ───────────────────────────────────────
  Serial.begin(115200);
  Serial.println("\nStarting Universal Space Pointer...");

  // ── 2. FreeRTOS Primitives ─────────────────────────────────────────
  stateMutex    = xSemaphoreCreateMutex();
  i2cMutex      = xSemaphoreCreateMutex();
  sgp4Mutex     = xSemaphoreCreateMutex();
  tleFetchMutex = xSemaphoreCreateMutex();
  if (stateMutex == NULL || i2cMutex == NULL ||
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
  const bool encoderDetected = encoder.isConnected();
  if (!encoderDetected) {
      Serial.println("WARNING: AS5600 not found — motor control will stay disabled");
  }

  // ── BNO055 IMU ─────────────────────────────────────────────────────
  const bool bnoDetected = bno.begin();
  if (!bnoDetected) {
      Serial.println("WARNING: BNO055 not found — motor control will stay disabled");
  }

  sensorsReady = encoderDetected && bnoDetected;

  // ── 4. Network & Web Services ──────────────────────────────────────
  WiFiManager wm;
  if (!wm.autoConnect("SpaceTracker_AP", SETUP_AP_PASSWORD)) {
      Serial.println("ERROR: Wi-Fi connection/configuration failed; motors remain disabled.");
      disableMotors();
      while (1) { delay(1000); }
  }
  if (!updateIPLocation()) {
      Serial.println("WARNING: IP geolocation failed; use Sync Phone GPS before tracking.");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
      if (!r->authenticate(CONTROL_USER, CONTROL_PASSWORD)) {
          r->requestAuthentication();
          return;
      }
      r->send(200, "text/html", index_html);
  });

  server.on("/updateLoc", HTTP_POST, [](AsyncWebServerRequest *r){
      if (!r->authenticate(CONTROL_USER, CONTROL_PASSWORD)) {
          r->requestAuthentication();
          return;
      }
      if (!hasValidMutationRequest(r)) {
          r->send(403, "text/plain", "Forbidden: missing same-origin request header.");
          return;
      }
      double lat, lon, alt;
      if (!parseDoubleArg(r, "lat", lat) || !parseDoubleArg(r, "lon", lon) ||
          !parseDoubleArg(r, "alt", alt) ||
          !TrackingMath::validObserver(lat, lon, alt)) {
          r->send(400, "text/plain", "Invalid location. Check latitude, longitude and altitude.");
          return;
      }
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
          r->send(503, "text/plain", "System busy; retry location update.");
          return;
      }
      sys.lat = lat; sys.lon = lon; sys.alt = alt;
      systemReadiness.locationReady = true;
      advanceConfigurationLocked();
      const TargetState target = current;
      xSemaphoreGive(stateMutex);
      if (target.type == TYPE_SATELLITE) {
          if (!startTLEFetchTask()) {
              r->send(503, "text/plain", "Could not start TLE refresh; motors remain disabled.");
              return;
          }
      }
      r->send(202, "text/plain", "Location accepted.");
  });

  server.on("/set", HTTP_POST, [](AsyncWebServerRequest *r){
      if (!r->authenticate(CONTROL_USER, CONTROL_PASSWORD)) {
          r->requestAuthentication();
          return;
      }
      if (!hasValidMutationRequest(r)) {
          r->send(403, "text/plain", "Forbidden: missing same-origin request header.");
          return;
      }
      if (!r->hasArg("type") || !r->hasArg("val")) {
          r->send(400, "text/plain", "Missing target type or value.");
          return;
      }
      const String typeArg = r->arg("type");
      const String valArg = r->arg("val");
      long type = 0, val = 0;
      if (!TrackingMath::parseLongInteger(typeArg.c_str(), type) ||
          (type != 0 && type != 1)) {
          r->send(400, "text/plain", "Invalid target type.");
          return;
      }
      if (!TrackingMath::parseLongInteger(valArg.c_str(), val)) {
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

      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
          r->send(503, "text/plain", "Target controller busy; retry.");
          return;
      }
      current = next;
      motorTarget.az = 0;
      motorTarget.el = 0;
      advanceConfigurationLocked();
      xSemaphoreGive(stateMutex);

      if (next.type == TYPE_SATELLITE) {
          if (!startTLEFetchTask()) {
              r->send(503, "text/plain", "Could not start TLE download; motors remain disabled.");
              return;
          }
      }
      r->send(202, "text/plain", "Target accepted.");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *r){
      TargetState target = {};
      TelemetrySnapshot data = {};
      // Read target + telemetry as one versioned snapshot. A target switch or
      // location update can otherwise pair the new name with old coordinates,
      // distances, or pass times. Return a temporary response until PhysicsTask
      // publishes telemetry for the exact current target/location pair.
      if (!getTelemetrySnapshot(target, data)) {
          r->send(503, "text/plain",
                  "Telemetry is updating for the selected target or location; please retry.");
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
  const bool timeSynced = (time(&now) >= 1000000000);
  if (timeSynced) {
      Serial.println("\nTime synchronized (UTC)!");
  } else {
      Serial.println("\nWARNING: NTP sync timed out; motor tracking remains disabled.");
  }

  // Load the default ISS TLE before the physics task can use the SGP4 object.
  const ConfigurationSnapshot initialConfig = getConfigurationSnapshot();
  if (timeSynced && initialConfig.locationReady &&
      TrackingMath::validObserver(initialConfig.lat, initialConfig.lon,
                                  initialConfig.alt)) {
      fetchTLE();
  }

  // ── 6. Task Creation ───────────────────────────────────────────────
  // WiFi stack runs on Core 0 — put Physics on Core 1 to avoid contention
  if (xTaskCreatePinnedToCore(PhysicsTask, "Physics", 15000, NULL, 2, NULL, 1) != pdPASS) {
      Serial.println("CRITICAL: Physics task creation failed.");
      disableMotors();
  }
  if (xTaskCreatePinnedToCore(MotorTask, "Motors", 8000, NULL, 3,
                              &motorTaskHandle, 0) != pdPASS) {
      Serial.println("CRITICAL: Motor task creation failed.");
      disableMotors();
  }

}

void loop() {
  vTaskDelay(portMAX_DELAY);
}



void advanceConfigurationLocked() {
  systemReadiness.invalidate();
  sys.nextPassValid = false;
  sys.nextPassAOS = 0;
  sys.nextPassLOS = 0;
  sys.nextPassMaxEl = 0;
}

ConfigurationSnapshot getConfigurationSnapshot() {
  ConfigurationSnapshot result = {};
  if (stateMutex != NULL &&
      xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      result.target = current;
      result.generation = systemReadiness.generation;
      result.locationReady = systemReadiness.locationReady;
      result.lat = sys.lat;
      result.lon = sys.lon;
      result.alt = sys.alt;
      xSemaphoreGive(stateMutex);
  }
  return result;
}

bool getTelemetrySnapshot(TargetState &target, TelemetrySnapshot &snapshot) {
  if (stateMutex == NULL ||
      xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
      return false;
  }

  const bool ready = systemReadiness.calculationReady;
  if (ready) {
      target = current;
      snapshot.targetAz = sys.targetAz;
      snapshot.targetEl = sys.targetEl;
      snapshot.satLat = sys.satLat;
      snapshot.satLon = sys.satLon;
      snapshot.satSpeed = sys.satSpeed;
      snapshot.satAltKm = sys.satAltKm;
      snapshot.satDist = sys.satDist;
      snapshot.nextPassValid = systemReadiness.passReady && sys.nextPassValid;
      snapshot.nextPassAOS = snapshot.nextPassValid ? sys.nextPassAOS : 0;
      snapshot.nextPassLOS = snapshot.nextPassValid ? sys.nextPassLOS : 0;
      snapshot.nextPassMaxEl = snapshot.nextPassValid ? sys.nextPassMaxEl : 0;
  }

  xSemaphoreGive(stateMutex);
  return ready;
}

bool isTimeReady() {
  time_t now = 0;
  time(&now);
  return now >= 1000000000;
}

// Lock order is always stateMutex -> sgp4Mutex. This makes checking the TLE
// generation and acquiring the shared SGP4 model one atomic operation.
bool lockTleForGeneration(uint32_t generation) {
  if (stateMutex == NULL ||
      xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
      return false;
  }

  const bool ready = systemReadiness.isCurrent(generation) &&
                     systemReadiness.tleReady;
  const bool locked = ready &&
      xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(100)) == pdTRUE;
  xSemaphoreGive(stateMutex);
  return locked;
}



bool hasValidMutationRequest(AsyncWebServerRequest *request) {
  // Cross-origin HTML forms cannot set this custom header. Cross-origin fetch
  // requests must pass a CORS preflight, which this device does not enable.
  return request->hasHeader("X-SpacePointer-Request") &&
         request->getHeader("X-SpacePointer-Request")->value() == "1";
}

bool parseDoubleArg(AsyncWebServerRequest *request, const char *name, double &value) {
  if (!request->hasArg(name)) return false;
  const String arg = request->arg(name);
  return TrackingMath::parseFiniteDouble(arg.c_str(), value);
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

// The stop bit is latched by FreeRTOS until MotorTask consumes it. Only
// MotorTask writes motor GPIOs after setup.
void disableMotors() {
  if (motorTaskHandle != NULL) {
      xTaskNotify(motorTaskHandle, MOTOR_STOP_NOTIFICATION, eSetBits);
  }
}

// Configuration changes invalidate readiness before requesting TLE data.
bool startTLEFetchTask() {
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
void updateNextPass(const ConfigurationSnapshot &snapshot, time_t now) {
    const TargetState &target = snapshot.target;
    bool passValid = false;
    time_t aosTime = 0;
    time_t losTime = 0;
    double maxElevation = 0.0;

    if (target.type == TYPE_SATELLITE &&
        lockTleForGeneration(snapshot.generation)) {
        const bool predictionReady =
            satSgp4.initpredpoint((unsigned long)now, 0.0);
        passinfo overpass = {};
        passValid = predictionReady && satSgp4.nextpass(&overpass, 20, false, 20.0);
        if (passValid) {
            aosTime = (time_t)((overpass.jdstart - 2440587.5) * 86400.0);
            losTime = (time_t)((overpass.jdstop  - 2440587.5) * 86400.0);
            maxElevation = overpass.maxelevation;
        }
        xSemaphoreGive(sgp4Mutex);
    } else if (target.type == TYPE_PLANET) {
        struct tm utc;
        if (gmtime_r(&now, &utc) != nullptr) {
            astro_time_t start = Astronomy_MakeTime(
                utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                utc.tm_hour, utc.tm_min, (double)utc.tm_sec);
            const astro_observer_t observer = {snapshot.lat, snapshot.lon, snapshot.alt};
            const astro_search_result_t rise = Astronomy_SearchRiseSet(
                (astro_body_t)target.body, observer, DIRECTION_RISE, start, 1.0);
            if (rise.status == ASTRO_SUCCESS) {
                const astro_time_t afterRise =
                    Astronomy_AddDays(rise.time, 1.0 / 86400.0);
                const astro_search_result_t set = Astronomy_SearchRiseSet(
                    (astro_body_t)target.body, observer, DIRECTION_SET, afterRise, 1.0);
                if (set.status == ASTRO_SUCCESS &&
                    TrackingMath::chronologicalEventPair(rise.time.ut, set.time.ut)) {
                    aosTime = (time_t)(rise.time.ut * 86400.0 + 946728000.0);
                    losTime = (time_t)(set.time.ut * 86400.0 + 946728000.0);
                    passValid = losTime > aosTime;
                }
            }
        }
    }

    if (stateMutex != NULL &&
        xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (systemReadiness.isCurrent(snapshot.generation)) {
            sys.nextPassValid = passValid;
            sys.nextPassAOS = aosTime;
            sys.nextPassLOS = losTime;
            sys.nextPassMaxEl = maxElevation;
            systemReadiness.publishPass(snapshot.generation);
        }
        xSemaphoreGive(stateMutex);
    }
}



void PhysicsTask(void * p) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
      const ConfigurationSnapshot snapshot = getConfigurationSnapshot();
      if (snapshot.generation == 0 || !snapshot.locationReady ||
          !TrackingMath::validObserver(snapshot.lat, snapshot.lon, snapshot.alt) ||
          !isTimeReady()) {
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }

      const TargetState &target = snapshot.target;
      const uint32_t generation = snapshot.generation;
      time_t now = 0;
      time(&now);
      if (now < 1000000000) {
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }

      static uint32_t lastPassCalc = 0;
      if (millis() - lastPassCalc > 30000) {
          updateNextPass(snapshot, now);
          lastPassCalc = millis();
      }

      double targetAz = 0.0, targetEl = 0.0;
      double satLat = 0.0, satLon = 0.0;
      double satSpeed = 0.0, satAltKm = 0.0, satDist = 0.0;
      bool calculationOK = false;

      if (target.type == TYPE_SATELLITE) {
          if (lockTleForGeneration(generation)) {
              satSgp4.findsat((unsigned long)now);
              targetAz = satSgp4.satAz;
              targetEl = satSgp4.satEl;
              satLat = satSgp4.satLat;
              satLon = satSgp4.satLon;
              satAltKm = satSgp4.satAlt;
              satDist = satSgp4.satDist;

              const double tsinceMinutes =
                  (satSgp4.satJd - satSgp4.satrec.jdsatepoch) * 1440.0;
              double position[3], velocity[3];
              if (sgp4(wgs84, satSgp4.satrec, tsinceMinutes, position, velocity)) {
                  satSpeed = sqrt(velocity[0] * velocity[0] +
                                  velocity[1] * velocity[1] +
                                  velocity[2] * velocity[2]);
                  calculationOK = isfinite(targetAz) && isfinite(targetEl) &&
                                  isfinite(satSpeed) && isfinite(satAltKm) &&
                                  isfinite(satDist);
              }
              xSemaphoreGive(sgp4Mutex);
          }
      } else {
          struct tm utc;
          if (gmtime_r(&now, &utc) != nullptr) {
              astro_time_t aTime = Astronomy_MakeTime(
                  utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                  utc.tm_hour, utc.tm_min, (double)utc.tm_sec);
              const astro_observer_t observer = {snapshot.lat, snapshot.lon, snapshot.alt};
              astro_equatorial_t equatorial = Astronomy_Equator(
                  (astro_body_t)target.body, &aTime, observer,
                  EQUATOR_OF_DATE, ABERRATION);
              if (equatorial.status == ASTRO_SUCCESS) {
                  astro_horizon_t horizon = Astronomy_Horizon(
                      &aTime, observer, equatorial.ra, equatorial.dec, REFRACTION_NORMAL);
                  const double distanceKm = equatorial.dist * 149597870.7;
                  targetAz = horizon.azimuth;
                  targetEl = horizon.altitude;
                  satAltKm = distanceKm;
                  satDist = distanceKm;
                  calculationOK = isfinite(targetAz) && isfinite(targetEl) &&
                                  isfinite(distanceKm);
              }
          }
      }

      // Reject calculations if either target or observer changed while working.
      if (calculationOK && stateMutex != NULL &&
          xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          if (systemReadiness.isCurrent(generation)) {
              sys.targetAz = targetAz;
              sys.targetEl = targetEl;
              sys.satLat = satLat;
              sys.satLon = satLon;
              sys.satSpeed = satSpeed;
              sys.satAltKm = satAltKm;
              sys.satDist = satDist;
              motorTarget.az = (float)targetAz;
              motorTarget.el = (float)targetEl;
              systemReadiness.publishCalculation(generation);
          }
          xSemaphoreGive(stateMutex);
      }

      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
  }
}



void MotorTask(void * p) {
  TickType_t lastWake = xTaskGetTickCount();
  bool driversEnabled = false;

  for (;;) {
      uint32_t notifications = 0;
      xTaskNotifyWait(0, MOTOR_STOP_NOTIFICATION, &notifications, 0);
      const bool stopRequested = (notifications & MOTOR_STOP_NOTIFICATION) != 0;
      if (stopRequested && driversEnabled) {
          digitalWrite(AZ_EN_PIN, HIGH);
          digitalWrite(EL_EN_PIN, HIGH);
          driversEnabled = false;
      }

      float targetAz = 0.0f, targetEl = 0.0f;
      TargetState activeTarget = {};
      uint32_t activeGeneration = 0;
      bool targetRead = false;
      bool observerReady = false;
      bool calculationReady = false;
      bool tleReadyForTarget = false;

      if (stateMutex != NULL &&
          xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          activeTarget = current;
          activeGeneration = systemReadiness.generation;
          targetAz = motorTarget.az;
          targetEl = motorTarget.el;
          observerReady = systemReadiness.locationReady &&
                           TrackingMath::validObserver(sys.lat, sys.lon, sys.alt);
          calculationReady = systemReadiness.calculationReady;
          tleReadyForTarget = activeTarget.type != TYPE_SATELLITE ||
                              systemReadiness.tleReady;
          targetRead = true;
          xSemaphoreGive(stateMutex);
      }

      if (!TrackingMath::motorControlReady(
              targetRead, isTimeReady(), observerReady, calculationReady,
              tleReadyForTarget, sensorsReady, targetAz, targetEl)) {
          if (driversEnabled) {
              digitalWrite(AZ_EN_PIN, HIGH);
              digitalWrite(EL_EN_PIN, HIGH);
              driversEnabled = false;
          }
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }

      if (stopRequested) {
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }

      sensors_event_t event = {};
      uint16_t encoderRaw = 0;
      uint8_t systemCal = 0, gyroCal = 0, accelCal = 0, magCal = 0;
      bool headingRead = false;
      bool encoderHealthy = false;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          headingRead = bno.getEvent(&event);
          bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
          encoderRaw = encoder.rawAngle();
          encoderHealthy = encoder.lastError() == AS5600_OK;
          xSemaphoreGive(i2cMutex);
      }

      const float headingNow = event.orientation.x;
      if (!TrackingMath::validMotorSensorSample(
              headingRead, encoderHealthy, systemCal, magCal,
              headingNow, encoderRaw)) {
          if (driversEnabled) {
              digitalWrite(AZ_EN_PIN, HIGH);
              digitalWrite(EL_EN_PIN, HIGH);
              driversEnabled = false;
          }
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }

      float elNow = NAN;
      if (!TrackingMath::encoderElevation(encoderRaw, ENCODER_ZERO_RAW,
                                         ENCODER_DIRECTION, elNow)) {
          if (driversEnabled) {
              digitalWrite(AZ_EN_PIN, HIGH);
              digitalWrite(EL_EN_PIN, HIGH);
              driversEnabled = false;
          }
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }

      const float headingTrue = TrackingMath::trueHeading(
          headingNow, MAGNETIC_DECLINATION_DEG);
      const int azSteps = pidOutput(
          pAz, TrackingMath::azimuthError(targetAz, headingTrue));
      const int elSteps = pidOutput(
          pEl, TrackingMath::elevationError(targetEl, elNow));
      const bool azFwd = azSteps > 0;
      const bool elFwd = elSteps > 0;
      const int azN = abs(azSteps);
      const int elN = abs(elSteps);
      const int maxN = max(azN, elN);

      if (stateMutex != NULL &&
          xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          uint32_t finalNotifications = 0;
          xTaskNotifyWait(0, MOTOR_STOP_NOTIFICATION, &finalNotifications, 0);
          const bool commandStillCurrent =
              systemReadiness.isCurrent(activeGeneration) &&
              systemReadiness.calculationReady &&
              (activeTarget.type != TYPE_SATELLITE || systemReadiness.tleReady) &&
              (finalNotifications & MOTOR_STOP_NOTIFICATION) == 0;
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
          xSemaphoreGive(stateMutex);
      } else if (driversEnabled) {
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
                if (stateMutex != NULL &&
                    xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    sys.lat = lat;
                    sys.lon = lon;
                    sys.alt = 50.0;
                    systemReadiness.locationReady = true;
                    advanceConfigurationLocked();
                    xSemaphoreGive(stateMutex);
                    valid = true;
                }
            }
        }
    }
    http.end();
    return valid;
}



void fetchTLE() {
    if (tleFetchMutex == NULL ||
        xSemaphoreTake(tleFetchMutex, pdMS_TO_TICKS(20000)) != pdTRUE) {
        disableMotors();
        return;
    }

    const ConfigurationSnapshot requested = getConfigurationSnapshot();
    bool success = false;

    if (requested.generation != 0 &&
        requested.target.type == TYPE_SATELLITE &&
        TrackingMath::validObserver(requested.lat, requested.lon, requested.alt) &&
        requested.target.noradID > 0) {
        HTTPClient http;
        http.setTimeout(12000);
        const String url = "https://tle.ivanstanojevic.me/api/tle/" +
                           String(requested.target.noradID);
        if (http.begin(url)) {
            const int status = http.GET();
            if (status == HTTP_CODE_OK) {
                StaticJsonDocument<1024> doc;
                const DeserializationError error = deserializeJson(doc, http.getString());
                const char *rawLine1 = doc["line1"].as<const char *>();
                const char *rawLine2 = doc["line2"].as<const char *>();

                if (!error && TrackingMath::validTlePair(
                        rawLine1, rawLine2, requested.target.noradID)) {
                    char line1[71] = {};
                    char line2[71] = {};
                    strlcpy(line1, rawLine1, sizeof(line1));
                    strlcpy(line2, rawLine2, sizeof(line2));

                    // Global lock order: stateMutex -> sgp4Mutex.
                    if (stateMutex != NULL &&
                        xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        const bool stillCurrent =
                            systemReadiness.isCurrent(requested.generation) &&
                            current.type == TYPE_SATELLITE &&
                            current.noradID == requested.target.noradID;
                        bool sgp4Locked = false;
                        if (stillCurrent) {
                            sgp4Locked =
                                xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(500)) == pdTRUE;
                        }
                        if (stillCurrent && sgp4Locked) {
                            satSgp4.site(requested.lat, requested.lon, requested.alt);
                            satSgp4.init(requested.target.name, line1, line2);
                            success = systemReadiness.publishTle(requested.generation);
                        }
                        if (sgp4Locked) xSemaphoreGive(sgp4Mutex);
                        xSemaphoreGive(stateMutex);
                    }
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
        bool requestStillCurrent = false;
        if (stateMutex != NULL &&
            xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            requestStillCurrent =
                requested.generation != 0 &&
                systemReadiness.isCurrent(requested.generation) &&
                current.type == TYPE_SATELLITE &&
                current.noradID == requested.target.noradID;
            if (requestStillCurrent) {
                systemReadiness.clearResults();
                sys.nextPassValid = false;
            }
            xSemaphoreGive(stateMutex);
        }

        if (requestStillCurrent) {
            disableMotors();
            Serial.println("WARNING: TLE update failed; satellite motors remain disabled.");
        } else {
            Serial.println("INFO: Discarded stale TLE result after target/location changed.");
        }
    }

    xSemaphoreGive(tleFetchMutex);
}

