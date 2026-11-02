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
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tracking_math.h"
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
#define MAX_STEPS_PER_TICK  80
// Two axes are stepped sequentially; 80 leaves time for task/sensor overhead.

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

  // display data
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
struct PID {
  float integral = 0, lastErr = 0;
  float kp, ki, kd, deadband;
};
struct MotorTarget { float az = 0; float el = 0; } motorTarget;

PID pidAz = {0.0f, 0.0f, 2.5f, 0.02f, 0.4f, 1.5f};
PID pidEl = {0.0f, 0.0f, 3.0f, 0.03f, 0.5f, 1.0f};
SemaphoreHandle_t targetMutex;   // protects current and motorTarget
SemaphoreHandle_t i2cMutex;      // protects the shared I2C bus
SemaphoreHandle_t sysMutex;      // protects observer and telemetry state
SemaphoreHandle_t sgp4Mutex;     // protects the shared SGP4 instance
SemaphoreHandle_t tleFetchMutex;

volatile bool bnoReady = false;
volatile bool encoderReady = false;
volatile bool observerReady = false;
volatile bool timeReady = false;
volatile bool tleReady = false;
volatile bool targetCalculationReady = false;
volatile bool pidResetRequested = false;

// Change both credentials before deploying outside a trusted test network.
static const char *CONTROL_USER = "admin";
static const char *CONTROL_PASSWORD = "spacepointer-change-me";
static const char *SETUP_AP_PASSWORD = "SpacePointerSetup";

struct TelemetrySnapshot {
  double lat, lon, alt;
  double targetAz, targetEl;
  double satLat, satLon;
  double satSpeed, satAltKm, satDist;
  bool nextPassValid;
  time_t nextPassAOS, nextPassLOS;
  double nextPassMaxEl;
};
// ── AS5600 encoder ──────────────────────────────────────────────────────
AS5600 encoder;   // default I2C address 0x36, shares bus with BNO055 (0x28)
// LCD — try 0x27 first, some boards use 0x3F
// Change to LiquidCrystal_I2C lcd(0x3F, 16, 2) if 0x27 doesn't work
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool lcdFound = false;   // set in setup(), checked before creating DisplayTask


void DisplayTask(void * p);
bool updateIPLocation();
void fetchTLE();
bool startTLEFetchTask();
TargetState getTargetSnapshot();
TelemetrySnapshot getTelemetrySnapshot();
bool getObserverSnapshot(double &lat, double &lon, double &alt);
bool parseDoubleArg(AsyncWebServerRequest *request, const char *name, double &value);
void disableMotors();
void resetPIDs();
const char *planetName(int body);
void PhysicsTask(void * p);
void stepMotor(int stepPin, int dirPin, bool forward);
int pidOutput(PID &p, float error, float dt);
float wrapAngle(float e);
void MotorTask(void * p);


const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <head>
      <title>Universal Space Pointer</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
      <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
      <style>
        /* Ensure padding and borders don't increase element widths */
        * { box-sizing: border-box; }
        
        body { font-family: 'Segoe UI', Arial; text-align: center; background: #121212; color: white; margin: 0; padding: 15px; }
        
        /* Map uses 100% width of its parent card to avoid overflow */
        #map { height: 320px; width: 100%; margin: 15px 0; border-radius: 12px; border: 2px solid #444; }
        
        /* Width 100% ensures it scales down on mobile, while max-width keeps it neat on PC */
        .card { background: #1e1e1e; padding: 20px; border-radius: 15px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); width: 100%; max-width: 500px; margin: auto; }
        
        .btn { padding: 10px 15px; margin: 5px; cursor: pointer; border-radius: 6px; border: none; background: #3498db; color: white; transition: 0.2s; }
        .btn:hover { background: #2980b9; }
        .input-group { margin: 20px 0; display: flex; justify-content: center; gap: 10px; flex-wrap: wrap; }
        
        /* Max-width prevents long selections from breaking mobile views */
        select, input { padding: 8px; border-radius: 5px; border: none; background: #333; color: white; max-width: 100%; }
        
        .data { color: #00ffcc; font-weight: bold; }
        .label { color: #888; font-size: 0.85em; }
    
        /* ── tracking name banner ── */
        #target-name {
          font-size: 1.6em; font-weight: bold; letter-spacing: 2px;
          color: #00ffcc; margin: 0 0 12px; text-transform: uppercase;
          word-wrap: break-word; /* Prevents long names from breaking layout */
        }
    
        /* ── stats grid ── */
        .stats-grid {
          display: grid; grid-template-columns: repeat(3, 1fr);
          gap: 10px; margin: 12px 0;
        }
        .stat-box {
          background: #2a2a2a; border-radius: 10px; padding: 10px 6px;
          border: 1px solid #333;
          min-width: 0; /* Allows grid items to shrink properly if needed */
        }
        .stat-val { font-size: 1.2em; color: #00ffcc; font-weight: bold; word-wrap: break-word; }
        .stat-lbl { font-size: 0.72em; color: #888; margin-top: 3px; }
    
        /* ── next pass strip ── */
        #next-pass-bar {
          background: #2a2a2a; border-radius: 10px; padding: 10px 14px;
          margin: 10px 0; font-size: 0.9em; border: 1px solid #333;
          display: flex; justify-content: space-between; align-items: center;
          flex-wrap: wrap; /* Allows stacking on extremely narrow screens */
          gap: 10px;
        }
        #next-pass-bar .label { font-size: 0.8em; }
      </style>
    </head>
    <body>
      <div class="card">
    
        <p id="target-name">---</p>
    
        <div id="map"></div>
    
        <p>
          <span class="label">Az</span> <span id="az" class="data">0</span>&deg;&nbsp;&nbsp;
          <span class="label">El</span> <span id="el" class="data">0</span>&deg;
        </p>
    
        <div class="stats-grid">
          <div class="stat-box">
            <div class="stat-val"><span id="speed">--</span></div>
            <div class="stat-lbl">km/s speed</div>
          </div>
          <div class="stat-box">
            <div class="stat-val"><span id="alt">--</span></div>
            <div class="stat-lbl" data-lbl="alt">km altitude</div>
          </div>
          <div class="stat-box">
            <div class="stat-val"><span id="dist">--</span></div>
            <div class="stat-lbl" data-lbl="dist">km distance</div>
          </div>
        </div>
    
        <div id="next-pass-bar">
          <span><span class="label" data-lbl="pass">Next pass</span> <span id="nextPass" class="data">--:--</span></span>
          <span><span class="label" data-lbl="lose">Max el</span> <span id="nextPassEl" class="data">--</span></span>
        </div>
    
        <button class="btn" style="background:#e67e22" onclick="syncGPS()">Sync Phone GPS</button>
    
        <div class="input-group">
          <select id="commonSats" onchange="if(this.value) setTarget(0, this.options[this.selectedIndex].text, this.value)">
            <option value="">-- Choose Satellite --</option>
            <option value="25544">ISS (Space Station)</option>
            <option value="20580">Hubble Telescope</option>
            <option value="48274">Tiangong (China)</option>
            <option value="25867">NOAA 15 (Weather)</option>
          </select>
          <button class="btn" onclick="setTarget(1, 'Moon', 10)">Moon</button>
          <button class="btn" onclick="setTarget(1, 'Mars', 3)">Mars</button>
          <button class="btn" onclick="setTarget(1, 'Jupiter', 4)">Jupiter</button>
          <button class="btn" onclick="setTarget(1, 'Saturn', 5)">Saturn</button>
        </div>
    
        <div class="input-group">
          <input type="number" id="norad" placeholder="Custom NORAD ID">
          <button class="btn" onclick="setCustomSat()">Track ID</button>
        </div>
      </div>
    
      <script>
        var map = L.map('map').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
        var marker = L.marker([0, 0]).addTo(map);
    
        function setTarget(type, name, val) {
          fetch('/set?type='+type+'&name='+encodeURIComponent(name)+'&val='+val)
            .then(function(r) { if (!r.ok) throw new Error('Target update failed: '+r.status); })
            .catch(function(e) { alert(e.message); });
        }
        function setCustomSat() {
          var id = document.getElementById('norad').value;
          if(id) setTarget(0, 'Custom', id);
        }
        function syncGPS() {
          navigator.geolocation.getCurrentPosition(function(p) {
            fetch('/updateLoc?lat='+p.coords.latitude+'&lon='+p.coords.longitude+'&alt='+(p.coords.altitude||50))
              .then(function(r) { if (!r.ok) throw new Error('Location update failed: '+r.status); alert("GPS Updated!"); })
              .catch(function(e) { alert(e.message); });
          });
        }
    
        setInterval(function() {
          fetch('/data').then(function(res) { return res.json(); }).then(function(d) {
            var isSat = d.type === 0;
    
            // name + pointing
            document.getElementById('target-name').innerText = d.name;
            document.getElementById('az').innerText = d.az.toFixed(2);
            document.getElementById('el').innerText = d.el.toFixed(2);
    
            // speed (same for both — always km/s)
            document.getElementById('speed').innerText = d.speed > 0 ? d.speed.toFixed(2) : '--';
    
            // altitude label: "km altitude" for sats, "km from Earth" for planets
            document.querySelector('[data-lbl="alt"]').innerText  = isSat ? 'km altitude' : 'km from observer';
            document.getElementById('alt').innerText = d.alt > 0 ? Math.round(d.alt) : '--';
    
            // distance label: "km distance" for sats, same "km from Earth" for planets
            document.querySelector('[data-lbl="dist"]').innerText = isSat ? 'km distance'   : 'km from observer';
            document.getElementById('dist').innerText = d.dist > 0 ? Math.round(d.dist) : '--';
    
            // next pass row: labels and value format differ by type
            document.querySelector('[data-lbl="pass"]').innerText = isSat ? 'Next pass' : 'Rises at';
            document.querySelector('[data-lbl="lose"]').innerText = isSat ? 'Max el'    : 'Sets at';
            document.getElementById('nextPass').innerText = d.nextPass || '--:--';
            if (isSat) {
              document.getElementById('nextPassEl').innerText = d.nextPassEl > 0 ? d.nextPassEl + '\u00b0' : '--';
            } else {
              document.getElementById('nextPassEl').innerText = d.nextPassLOS || '--:--';
            }
    
            // map: only move for satellites
            if (isSat) {
              marker.setLatLng([d.sLat, d.sLon]);
              map.panTo([d.sLat, d.sLon]);
            }
          });
        }, 2000);
      </script>
    </body>
  </html>
  )rawliteral";


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

  // ── 3. Hardware Initialization ─────────────────────────────────────
  pinMode(AZ_STEP_PIN, OUTPUT); pinMode(AZ_DIR_PIN, OUTPUT);
  pinMode(EL_STEP_PIN, OUTPUT); pinMode(EL_DIR_PIN, OUTPUT);
  pinMode(AZ_EN_PIN, OUTPUT);   digitalWrite(AZ_EN_PIN, HIGH); // disabled
  pinMode(EL_EN_PIN, OUTPUT);   digitalWrite(EL_EN_PIN, HIGH); // disabled
  pinMode(AZ_STEP_PIN, OUTPUT); digitalWrite(AZ_STEP_PIN, LOW);
  pinMode(EL_STEP_PIN, OUTPUT); digitalWrite(EL_STEP_PIN, LOW);

  // ESP32-S3 DevKitC-1 I2C mapping; all three peripherals share this bus.
  Wire.begin(8, 9); // SDA=GPIO8, SCL=GPIO9
  Wire.setClock(400000);

  // ── LCD detection ──────────────────────────────────────────────────
  // Probe the I2C bus for the LCD address before init.
  // Wire.endTransmission()==0 means a device ACK'd at that address.
  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) {
      lcdFound = true;
  } else {
      // Try the alternate common address
      Wire.beginTransmission(0x3F);
      if (Wire.endTransmission() == 0) {
          lcdFound = true;
          // Reinitialise the object at the correct address
          lcd = LiquidCrystal_I2C(0x3F, 16, 2);
      }
  }

  if (lcdFound) {
      lcd.init();
      lcd.backlight();
      lcd.setCursor(0, 0); lcd.print("Space Pointer");
      lcd.setCursor(0, 1); lcd.print("Booting...");
      Serial.println("LCD found and initialised");
  } else {
      Serial.println("WARNING: LCD not found — DisplayTask will be skipped");
  }

  // ── AS5600 encoder ─────────────────────────────────────────────────
  encoder.begin();
  encoderReady = encoder.isConnected();
  if (!encoderReady) {
      Serial.println("WARNING: AS5600 not found — motor control will stay disabled");
  } else {
      encoder.setDirection(AS5600_CLOCK_WISE);
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
      r->send(200, "text/html", index_html);
  });

  server.on("/updateLoc", HTTP_GET, [](AsyncWebServerRequest *r){
      if (!r->authenticate(CONTROL_USER, CONTROL_PASSWORD)) {
          r->requestAuthentication();
          return;
      }
      double lat, lon, alt;
      if (!parseDoubleArg(r, "lat", lat) || !parseDoubleArg(r, "lon", lon) ||
          !parseDoubleArg(r, "alt", alt) || lat < -90.0 || lat > 90.0 ||
          lon < -180.0 || lon > 180.0 || alt < -500.0 || alt > 10000.0) {
          r->send(400, "text/plain", "Invalid location. Check latitude, longitude and altitude.");
          return;
      }
      if (xSemaphoreTake(sysMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
          r->send(503, "text/plain", "System busy; retry location update.");
          return;
      }
      sys.lat = lat; sys.lon = lon; sys.alt = alt;
      xSemaphoreGive(sysMutex);
      observerReady = true;
      targetCalculationReady = false;

      TargetState target = getTargetSnapshot();
      if (target.type == TYPE_SATELLITE) {
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
      if (end == typeArg.c_str() || *end != '\\0' || (type != 0 && type != 1)) {
          r->send(400, "text/plain", "Invalid target type.");
          return;
      }
      end = nullptr;
      long val = strtol(valArg.c_str(), &end, 10);
      if (end == valArg.c_str() || *end != '\\0') {
          r->send(400, "text/plain", "Invalid target ID.");
          return;
      }

      TargetState next = {};
      if (type == TYPE_SATELLITE) {
          if (val < 1 || val > 999999) {
              r->send(400, "text/plain", "NORAD ID is outside the accepted range.");
              return;
          }
          next.type = TYPE_SATELLITE;
          next.noradID = val;
          snprintf(next.name, sizeof(next.name), "NORAD %ld", val);
      } else {
          const char *name = planetName((int)val);
          if (name == nullptr) {
              r->send(400, "text/plain", "Unsupported planetary body.");
              return;
          }
          next.type = TYPE_PLANET;
          next.body = (int)val;
          strlcpy(next.name, name, sizeof(next.name));
      }

      if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
          r->send(503, "text/plain", "Target controller busy; retry.");
          return;
      }
      current = next;
      motorTarget.az = 0;
      motorTarget.el = 0;
      targetCalculationReady = false;
      xSemaphoreGive(targetMutex);
      pidResetRequested = true;

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
      const TelemetrySnapshot data = getTelemetrySnapshot();
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

  if (lcdFound) {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("WiFi OK");
      lcd.setCursor(0, 1); lcd.print("Time synced!");
      delay(1000);
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

  // Only create DisplayTask if an LCD was actually found
  if (lcdFound) {
      xTaskCreatePinnedToCore(DisplayTask, "Display", 4000, NULL, 1, NULL, 0);
  } else {
      Serial.println("DisplayTask skipped — no LCD");
  }
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}



TargetState getTargetSnapshot() {
  TargetState result = {};
  if (targetMutex != NULL && xSemaphoreTake(targetMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      result = current;
      xSemaphoreGive(targetMutex);
  }
  return result;
}

TelemetrySnapshot getTelemetrySnapshot() {
  TelemetrySnapshot result = {};
  if (sysMutex != NULL && xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      result.lat = sys.lat; result.lon = sys.lon; result.alt = sys.alt;
      result.targetAz = sys.targetAz; result.targetEl = sys.targetEl;
      result.satLat = sys.satLat; result.satLon = sys.satLon;
      result.satSpeed = sys.satSpeed; result.satAltKm = sys.satAltKm;
      result.satDist = sys.satDist;
      result.nextPassValid = sys.nextPassValid;
      result.nextPassAOS = sys.nextPassAOS; result.nextPassLOS = sys.nextPassLOS;
      result.nextPassMaxEl = sys.nextPassMaxEl;
      xSemaphoreGive(sysMutex);
  }
  return result;
}

bool getObserverSnapshot(double &lat, double &lon, double &alt) {
  if (sysMutex == NULL || xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
  lat = sys.lat; lon = sys.lon; alt = sys.alt;
  xSemaphoreGive(sysMutex);
  return isfinite(lat) && isfinite(lon) && isfinite(alt) &&
         lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

bool parseDoubleArg(AsyncWebServerRequest *request, const char *name, double &value) {
  if (!request->hasArg(name)) return false;
  const String arg = request->arg(name);
  char *end = nullptr;
  value = strtod(arg.c_str(), &end);
  return end != arg.c_str() && *end == '\\0' && isfinite(value);
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

void disableMotors() {
  digitalWrite(AZ_EN_PIN, HIGH);
  digitalWrite(EL_EN_PIN, HIGH);
}

void resetPIDs() {
  pidAz.integral = 0; pidAz.lastErr = 0;
  pidEl.integral = 0; pidEl.lastErr = 0;
}

bool startTLEFetchTask() {
  tleReady = false;
  targetCalculationReady = false;
  return xTaskCreate([](void *) {
      fetchTLE();
      vTaskDelete(NULL);
  }, "TLE", 10000, NULL, 1, NULL) == pdPASS;
}

float wrapAngle(float e) {
  return TrackingMath::wrapAngle(e);
}

int pidOutput(PID &p, float error, float dt) {
    if (!isfinite(error) || !isfinite(dt) || dt <= 0.0f) {
        p.integral = 0; p.lastErr = 0;
        return 0;
    }
    if (fabsf(error) < p.deadband) {
        p.integral = 0;
        p.lastErr = error;
        return 0;
    }
    p.integral  = constrain(p.integral + error * dt, -40.f, 40.f);
    float deriv = (error - p.lastErr) / dt;
    p.lastErr   = error;
    float out   = p.kp * error + p.ki * p.integral + p.kd * deriv;
    return (int)constrain(out, -(float)MAX_STEPS_PER_TICK, (float)MAX_STEPS_PER_TICK);
}

void PhysicsTask(void * p) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
      if (!timeReady || !observerReady) {
          targetCalculationReady = false;
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }

      const TargetState target = getTargetSnapshot();
      double lat, lon, alt;
      if (!getObserverSnapshot(lat, lon, alt)) {
          targetCalculationReady = false;
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }

      time_t now;
      time(&now);
      if (now < 1000000000) {
          targetCalculationReady = false;
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
          continue;
      }

      double targetAz = 0.0, targetEl = 0.0;
      double satLat = 0.0, satLon = 0.0;
      double satSpeed = 0.0, satAltKm = 0.0, satDist = 0.0;
      bool calculationOK = false;

      if (target.type == TYPE_SATELLITE) {
          if (tleReady && xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
              satSgp4.findsat((unsigned long)now);
              const double lat1 = satSgp4.satLat;
              const double lon1 = satSgp4.satLon;
              const double alt1 = satSgp4.satAlt;
              const double dist1 = satSgp4.satDist;
              targetAz = satSgp4.satAz;
              targetEl = satSgp4.satEl;

              satSgp4.findsat((unsigned long)(now + 1));
              const double dlat = (satSgp4.satLat - lat1) * DEG_TO_RAD * 6371.0;
              const double dlon = wrapAngle((float)(satSgp4.satLon - lon1)) *
                                  DEG_TO_RAD * 6371.0 * cos(lat1 * DEG_TO_RAD);
              const double dalt = satSgp4.satAlt - alt1;
              satSpeed = sqrt(dlat*dlat + dlon*dlon + dalt*dalt);

              satSgp4.findsat((unsigned long)now);
              satLat = lat1;
              satLon = lon1;
              satAltKm = alt1;
              satDist = dist1;
              xSemaphoreGive(sgp4Mutex);
              calculationOK = isfinite(targetAz) && isfinite(targetEl) &&
                              isfinite(satSpeed) && isfinite(satAltKm) &&
                              isfinite(satDist);
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
              astro_horizon_t hor1 = Astronomy_Horizon(
                  &aTime, obs, equ1.ra, equ1.dec, REFRACTION_NORMAL);

              astro_time_t aTime2 = Astronomy_AddDays(aTime, 1.0 / 86400.0);
              astro_equatorial_t equ2 = Astronomy_Equator(
                  (astro_body_t)target.body, &aTime2, obs,
                  EQUATOR_OF_DATE, ABERRATION);
              astro_horizon_t hor2 = Astronomy_Horizon(
                  &aTime2, obs, equ2.ra, equ2.dec, REFRACTION_NORMAL);

              const double distanceKm = equ1.dist * 149597870.7;
              const double dDistance = (equ2.dist - equ1.dist) * 149597870.7;
              const double dAz = wrapAngle((float)(hor2.azimuth - hor1.azimuth)) *
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

      const TargetState latestTarget = getTargetSnapshot();
      const bool sameTarget = latestTarget.type == target.type &&
          ((target.type == TYPE_SATELLITE && latestTarget.noradID == target.noradID) ||
           (target.type == TYPE_PLANET && latestTarget.body == target.body));

      if (calculationOK && sameTarget &&
          xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          sys.targetAz = targetAz;
          sys.targetEl = targetEl;
          sys.satLat = satLat;
          sys.satLon = satLon;
          sys.satSpeed = satSpeed;
          sys.satAltKm = satAltKm;
          sys.satDist = satDist;
          xSemaphoreGive(sysMutex);

          if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
              motorTarget.az = (float)targetAz;
              motorTarget.el = (float)targetEl;
              xSemaphoreGive(targetMutex);
          }
          targetCalculationReady = true;
      } else {
          targetCalculationReady = false;
      }

      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
  }
}




void MotorTask(void * p) {
  const float DT = MOTOR_TASK_MS / 1000.0f;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
      float targetAz = 0.0f, targetEl = 0.0f;
      bool targetRead = false;
      if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          targetAz = motorTarget.az;
          targetEl = motorTarget.el;
          targetRead = true;
          xSemaphoreGive(targetMutex);
      }

      if (pidResetRequested) {
          resetPIDs();
          pidResetRequested = false;
      }
      const TargetState activeTarget = getTargetSnapshot();
      const bool orbitalDataReady = activeTarget.type != TYPE_SATELLITE || tleReady;
      if (!targetRead || !timeReady || !observerReady || !targetCalculationReady ||
          !orbitalDataReady || !bnoReady || !encoderReady || !isfinite(targetAz) ||
          !isfinite(targetEl) || targetEl < 0.0f || targetEl > 90.0f) {
          disableMotors();
          resetPIDs();
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }

      sensors_event_t event = {};
      uint16_t encoderRaw = 0;
      bool sensorsRead = false;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          bno.getEvent(&event);
          encoderRaw = encoder.getRawAngle();
          sensorsRead = encoder.isConnected();
          xSemaphoreGive(i2cMutex);
      }

      const float headingNow = event.orientation.x;
      if (!sensorsRead || !isfinite(headingNow) || headingNow < 0.0f ||
          headingNow >= 360.0f || encoderRaw > 4095) {
          disableMotors();
          resetPIDs();
          vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
          continue;
      }

      float elNow = encoderRaw * (360.0f / 4096.0f);
      if (elNow > 180.0f) elNow -= 360.0f;

      digitalWrite(AZ_EN_PIN, LOW);
      digitalWrite(EL_EN_PIN, LOW);

      const int azSteps = pidOutput(pidAz, wrapAngle(targetAz - headingNow), DT);
      const int elSteps = pidOutput(pidEl, wrapAngle(targetEl - elNow), DT);
      const bool azFwd = azSteps > 0;
      const bool elFwd = elSteps > 0;
      const int azN = abs(azSteps);
      const int elN = abs(elSteps);
      const int maxN = max(azN, elN);

      for (int i = 0; i < maxN; i++) {
          if (i < azN) stepMotor(AZ_STEP_PIN, AZ_DIR_PIN, azFwd);
          if (i < elN) stepMotor(EL_STEP_PIN, EL_DIR_PIN, elFwd);
      }
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_MS));
  }
}




void DisplayTask(void * p) {
  static uint32_t lastPassCalc = 0;
  static uint8_t screen = 0;
  char line0[17], line1[17];

  for (;;) {
      if (timeReady && millis() - lastPassCalc > 30000) {
          time_t now;
          time(&now);
          const TargetState target = getTargetSnapshot();
          bool passValid = false;
          time_t aosTime = 0, losTime = 0;
          double maxElevation = 0.0;

          if (target.type == TYPE_SATELLITE && tleReady &&
              xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
              satSgp4.initpredpoint((unsigned long)now, 0.0);
              passinfo overpass;
              passValid = satSgp4.nextpass(&overpass, 20);
              if (passValid) {
                  aosTime = (time_t)((overpass.jdstart - 2440587.5) * 86400.0);
                  losTime = (time_t)((overpass.jdstop  - 2440587.5) * 86400.0);
                  maxElevation = overpass.maxelevation;
              }
              xSemaphoreGive(sgp4Mutex);
          } else if (target.type == TYPE_PLANET) {
              double lat, lon, alt;
              struct tm utc;
              if (getObserverSnapshot(lat, lon, alt) && gmtime_r(&now, &utc) != nullptr) {
                  astro_time_t start = Astronomy_MakeTime(
                      utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                      utc.tm_hour, utc.tm_min, (double)utc.tm_sec);
                  const astro_observer_t obs = {lat, lon, alt};
                  const astro_search_result_t rise = Astronomy_SearchRiseSet(
                      (astro_body_t)target.body, obs, DIRECTION_RISE, start, 1.0);
                  const astro_search_result_t set = Astronomy_SearchRiseSet(
                      (astro_body_t)target.body, obs, DIRECTION_SET, start, 1.0);
                  if (rise.status == ASTRO_SUCCESS && set.status == ASTRO_SUCCESS) {
                      aosTime = (time_t)(rise.time.ut * 86400.0 + 946728000.0);
                      losTime = (time_t)(set.time.ut * 86400.0 + 946728000.0);
                      passValid = true;
                  }
              }
          }

          if (xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
              sys.nextPassValid = passValid;
              sys.nextPassAOS = aosTime;
              sys.nextPassLOS = losTime;
              sys.nextPassMaxEl = maxElevation;
              xSemaphoreGive(sysMutex);
          }
          lastPassCalc = millis();
      }

      const TargetState target = getTargetSnapshot();
      const TelemetrySnapshot data = getTelemetrySnapshot();
      if (screen == 0) {
          snprintf(line0, sizeof(line0), "%-4.4s Az:%-6.1f",
                   target.name, data.targetAz);
          if (data.targetEl >= 0) {
              snprintf(line1, sizeof(line1), "El:%-5.1f %5.0fkm",
                       data.targetEl, data.satAltKm);
          } else if (data.nextPassValid) {
              struct tm aos;
              if (gmtime_r(&data.nextPassAOS, &aos) != nullptr) {
                  snprintf(line1, sizeof(line1), "El:%-4.1f >%02d:%02dUTC",
                           data.targetEl, aos.tm_hour, aos.tm_min);
              } else {
                  snprintf(line1, sizeof(line1), "El:%-5.1f No pass", data.targetEl);
              }
          } else {
              snprintf(line1, sizeof(line1), "El:%-5.1f No pass", data.targetEl);
          }
      } else {
          snprintf(line0, sizeof(line0), "%-4.4s %6.2fkm/s",
                   target.name, data.satSpeed);
          if (target.type == TYPE_SATELLITE) {
              snprintf(line1, sizeof(line1), "Dist:%9.0fkm", data.satDist);
          } else {
              const double distMkm = data.satDist / 1000.0;
              if (distMkm < 10000.0) {
                  snprintf(line1, sizeof(line1), "Dist:%7.0f Mkm", distMkm);
              } else {
                  snprintf(line1, sizeof(line1), "Dist:    %5.2f AU",
                           data.satDist / 149597870.7);
              }
          }
      }

      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          lcd.setCursor(0, 0); lcd.print(line0);
          lcd.setCursor(0, 1); lcd.print(line1);
          xSemaphoreGive(i2cMutex);
      }
      screen = 1 - screen;
      vTaskDelay(pdMS_TO_TICKS(2000));
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
                    xSemaphoreGive(sysMutex);
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
        tleReady = false;
        targetCalculationReady = false;
        disableMotors();
        return;
    }

    tleReady = false;
    targetCalculationReady = false;
    const TargetState requested = getTargetSnapshot();
    double lat, lon, alt;
    bool success = false;

    if (requested.type == TYPE_SATELLITE &&
        getObserverSnapshot(lat, lon, alt) && requested.noradID > 0) {
        HTTPClient http;
        http.setTimeout(12000);
        const String url = "https://tle.ivanstanojevic.me/api/tle/" + String(requested.noradID);
        if (http.begin(url)) {
            const int status = http.GET();
            if (status == HTTP_CODE_OK) {
                StaticJsonDocument<1024> doc;
                const DeserializationError error = deserializeJson(doc, http.getString());
                const char *rawLine1 = doc["line1"] | (const char *)nullptr;
                const char *rawLine2 = doc["line2"] | (const char *)nullptr;

                if (!error && rawLine1 != nullptr && rawLine2 != nullptr &&
                    strlen(rawLine1) >= 69 && strlen(rawLine2) >= 69 &&
                    rawLine1[0] == '1' && rawLine2[0] == '2') {
                    char line1[71] = {};
                    char line2[71] = {};
                    strlcpy(line1, rawLine1, sizeof(line1));
                    strlcpy(line2, rawLine2, sizeof(line2));

                    const TargetState latest = getTargetSnapshot();
                    const bool stillCurrent = latest.type == TYPE_SATELLITE &&
                                              latest.noradID == requested.noradID;
                    if (stillCurrent &&
                        xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                        satSgp4.site(lat, lon, alt);
                        satSgp4.init(requested.name, line1, line2);
                        xSemaphoreGive(sgp4Mutex);
                        tleReady = true;
                        success = true;
                    }
                } else {
                    Serial.println("ERROR: TLE response was invalid or incomplete.");
                }
            } else {
                Serial.printf("ERROR: TLE server returned HTTP %d\\n", status);
            }
            http.end();
        }
    }

    if (!success) {
        tleReady = false;
        targetCalculationReady = false;
        disableMotors();
        Serial.println("WARNING: TLE update failed; satellite motors remain disabled.");
    }
    xSemaphoreGive(tleFetchMutex);
}
