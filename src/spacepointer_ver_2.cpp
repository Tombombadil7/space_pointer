/*
 * PROJECT: Universal Space Pointer v7.0
 * FEATURES: 
 * - Auto IP-Geolocation & Manual Phone GPS Sync
 * - Real-time Satellite (Sgp4) & Planetary Tracking (Astronomy Lib)
 * - Live Leaflet.js Map Interface
 * - Custom NORAD ID tracking
 * - IMU BNO055 Compensation (Yaw & Pitch) 
 */

// The TLS compatibility server includes its own WebServer-compatible API.
// WiFiManager normally includes Arduino's WebServer.h, whose HTTPUploadStatus,
// HTTPClientStatus and HTTPAuthMethod enums collide with the compatibility
// server's declarations. Reuse the compatible ESPWebServer class for the
// temporary Wi-Fi captive portal too, avoiding two conflicting definitions.
#include <ESPWebServer.hpp>
using WebServer = ESPWebServer;
#define WEBSERVER_H
#include <WiFiManager.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tracking_math.h"
#include <ESPWebServerSecure.hpp>
#include <SSLCert.hpp>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <new>
#include <mbedtls/oid.h>
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

// Two axes are stepped sequentially; 80 leaves time for task/sensor overhead.

// TLS-enabled server. Requests are handled synchronously in loop(); motor and physics work run in separate tasks.
ESPWebServerSecure server(443);
httpsserver::SSLCert localTlsCertificate;
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
// A deliberately simple proportional controller is enough for slow telescope tracking.
// It avoids derivative kick and integral wind-up when a new target is selected.
struct PController {
  float kp;
  float deadband;
};
struct MotorTarget { float az = 0; float el = 0; } motorTarget;

// Modest gains suit the small, repeated corrections used during normal tracking.
// A narrow deadband filters sensor noise without allowing a large pointing error.
PController pidAz = {2.5f, 0.05f};
PController pidEl = {3.0f, 0.05f};
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

// Other tasks may request a safe stop, but only MotorTask writes motor GPIOs.
// MotorTask consumes this flag and performs the actual ENABLE-pin changes.
volatile bool motorStopRequested = true;

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
bool updateIPLocation();
void fetchTLE();
bool startTLEFetchTask();
TargetState getTargetSnapshot();
TelemetrySnapshot getTelemetrySnapshot();
bool getObserverSnapshot(double &lat, double &lon, double &alt);
bool parseDoubleArg(const char *name, double &value);
bool configureHttpsCertificate();
bool authenticateControlRequest();
void disableMotors();
const char *planetName(int body);
void PhysicsTask(void * p);
void updateNextPass(const TargetState &target, time_t now,
                    double lat, double lon, double alt);
void stepMotor(int stepPin, int dirPin, bool forward);
int pidOutput(const PController &p, float error);
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
          // Geolocation requires HTTPS (a secure context) and explicit user permission.
          if (!navigator.geolocation) {
            alert('This browser does not provide geolocation.');
            return;
          }
          navigator.geolocation.getCurrentPosition(function(p) {
            var altitude = p.coords.altitude;
            if (altitude === null || !Number.isFinite(altitude)) altitude = 50;
            fetch('/updateLoc?lat='+p.coords.latitude+'&lon='+p.coords.longitude+'&alt='+altitude)
              .then(function(r) {
                if (!r.ok) throw new Error('Location update failed: '+r.status);
                alert('GPS Updated!');
              })
              .catch(function(e) { alert(e.message); });
          }, function(e) {
            alert('Could not read phone GPS: '+e.message+' (check HTTPS and location permission).');
          }, { enableHighAccuracy: true, timeout: 15000, maximumAge: 30000 });
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
            document.getElementById('alt').innerText = d.alt > 0 ? Math.round(d.alt) : '--';                // distance label: "km distance" for sats, same "km from Earth" for planets
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
  // Set a safe initial electrical state before the FreeRTOS motor task starts.
  // After startup, MotorTask is the only task allowed to write motor GPIOs.
  pinMode(AZ_EN_PIN, OUTPUT);   digitalWrite(AZ_EN_PIN, HIGH); // disabled
  pinMode(EL_EN_PIN, OUTPUT);   digitalWrite(EL_EN_PIN, HIGH); // disabled
  pinMode(AZ_STEP_PIN, OUTPUT); digitalWrite(AZ_STEP_PIN, LOW);
  pinMode(EL_STEP_PIN, OUTPUT); digitalWrite(EL_STEP_PIN, LOW);

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
  // Create or restore a stable self-signed certificate before starting HTTPS.
  if (!configureHttpsCertificate()) {
      Serial.println("CRITICAL: Could not create/load HTTPS certificate; motors remain disabled.");
      disableMotors();
      while (1) { delay(1000); }
  }

  // Advertise the memorable .local hostname on the local network.
  if (MDNS.begin("space-pointer")) {
      MDNS.addService("https", "tcp", 443);
      Serial.println("mDNS ready: https://space-pointer.local/");
  } else {
      Serial.println("WARNING: mDNS setup failed; use the IP URL printed below.");
  }

  observerReady = updateIPLocation();
  if (!observerReady) {
      Serial.println("WARNING: IP geolocation failed; use Sync Phone GPS before tracking.");
  }

  // HTTPS only: no unencrypted dashboard or control listener is started.
  server.on("/", HTTP_GET, [](){
      if (!authenticateControlRequest()) return;
      server.send_P(200, PSTR("text/html"), index_html);
  });

  server.on("/updateLoc", HTTP_GET, [](){
      if (!authenticateControlRequest()) return;
      double lat, lon, alt;
      if (!parseDoubleArg("lat", lat) || !parseDoubleArg("lon", lon) ||
          !parseDoubleArg("alt", alt) ||
          !TrackingMath::validObserver(lat, lon, alt)) {
          server.send(400, "text/plain", "Invalid location. Check latitude, longitude and altitude.");
          return;
      }
      if (xSemaphoreTake(sysMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
          server.send(503, "text/plain", "System busy; retry location update.");
          return;
      }
      sys.lat = lat; sys.lon = lon; sys.alt = alt;
      xSemaphoreGive(sysMutex);
      observerReady = true;
      targetCalculationReady = false;

      TargetState target = getTargetSnapshot();
      if (target.type == TYPE_SATELLITE) {
          tleReady = false;   // invalidate the old observer's SGP4 state immediately
          targetCalculationReady = false;
          if (!startTLEFetchTask()) {
              server.send(503, "text/plain", "Could not start TLE refresh; motors remain disabled.");
              return;
          }
      }
      server.send(202, "text/plain", "Location accepted.");
  });

  server.on("/set", HTTP_GET, [](){
      if (!authenticateControlRequest()) return;
      if (!server.hasArg("type") || !server.hasArg("val")) {
          server.send(400, "text/plain", "Missing target type or value.");
          return;
      }
      char *end = nullptr;
      const String typeArg = server.arg("type");
      const String valArg = server.arg("val");
      long type = strtol(typeArg.c_str(), &end, 10);
      if (end == typeArg.c_str() || *end != '\0' || (type != 0 && type != 1)) {
          server.send(400, "text/plain", "Invalid target type.");
          return;
      }
      end = nullptr;
      long val = strtol(valArg.c_str(), &end, 10);
      if (end == valArg.c_str() || *end != '\0') {
          server.send(400, "text/plain", "Invalid target ID.");
          return;
      }

      TargetState next = {};
      if (type == TYPE_SATELLITE) {
          if (!TrackingMath::validNoradId(val)) {
              server.send(400, "text/plain", "NORAD ID is outside the accepted range.");
              return;
          }
          next.type = TYPE_SATELLITE;
          next.noradID = val;
          String requestedName = server.arg("name");
          requestedName.trim();
          if (requestedName.length() > 0) {
              requestedName.toCharArray(next.name, sizeof(next.name));
              next.name[sizeof(next.name) - 1] = 0;
          } else {
              snprintf(next.name, sizeof(next.name), "NORAD %ld", val);
          }
      } else {
          if (!TrackingMath::supportedPlanetBody((int)val)) {
              server.send(400, "text/plain", "Unsupported planetary body.");
              return;
          }
          next.type = TYPE_PLANET;
          next.body = (int)val;
          strlcpy(next.name, planetName((int)val), sizeof(next.name));
      }

      if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
          server.send(503, "text/plain", "Target controller busy; retry.");
          return;
      }
      if (next.type == TYPE_SATELLITE) {
          tleReady = false;   // never track a newly selected NORAD ID with the previous TLE
      }
      targetCalculationReady = false;
      current = next;
      motorTarget.az = 0;
      motorTarget.el = 0;
      xSemaphoreGive(targetMutex);

      if (next.type == TYPE_SATELLITE) {
          if (!startTLEFetchTask()) {
              server.send(503, "text/plain", "Could not start TLE download; motors remain disabled.");
              return;
          }
      }
      server.send(202, "text/plain", "Target accepted.");
  });

  server.on("/data", HTTP_GET, [](){
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
      server.send(200, "application/json", out);
  });

  server.onNotFound([]() {
      server.send(404, "text/plain", "Not found.");
  });

  server.begin();
  Serial.println("HTTPS dashboard: https://space-pointer.local/ (if mDNS is supported)");
  Serial.print("HTTPS fallback URL: https://");
  Serial.print(WiFi.localIP());
  Serial.println("/");

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
  // The TLS server is synchronous: poll it often while the motor and physics
  // FreeRTOS tasks continue independently.
  server.handleClient();
  delay(2);
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
  return TrackingMath::validObserver(lat, lon, alt);
}

bool parseDoubleArg(const char *name, double &value) {
  if (!server.hasArg(name)) return false;
  const String arg = server.arg(name);
  char *end = nullptr;
  value = strtod(arg.c_str(), &end);
  return end != arg.c_str() && *end == '\0' && isfinite(value);
}

bool authenticateControlRequest() {
  // Keep the same credentials, now protected in transit by TLS.
  if (server.authenticate(CONTROL_USER, CONTROL_PASSWORD)) return true;
  server.requestAuthentication();
  return false;
}

// Generate a self-signed certificate with a Subject Alternative Name (SAN).
// Modern browsers validate the hostname against SAN; a matching Common Name alone
// is not enough. The DER buffers are allocated on the heap because TLS keeps them
// for the lifetime of the HTTPS server.
int createSpacePointerCertificate(httpsserver::SSLCert &certContext) {
  mbedtls_entropy_context entropy;
  mbedtls_ctr_drbg_context randomGenerator;
  mbedtls_pk_context privateKey;
  mbedtls_x509write_cert certificateWriter;
  unsigned char *temporaryKey = nullptr;
  unsigned char *savedKey = nullptr;
  unsigned char *temporaryCertificate = nullptr;
  unsigned char *savedCertificate = nullptr;
  int keyBytesWritten = 0;
  int certificateBytesWritten = 0;
  int result = 0;
#if MBEDTLS_VERSION_NUMBER < 0x03000000
  mbedtls_mpi serialNumber;
  mbedtls_mpi_init(&serialNumber);
#endif

  // DER-encoded GeneralNames -> dNSName [2] "space-pointer.local".
  const unsigned char subjectAltName[] = {
    0x30, 0x15, 0x82, 0x13,
    's','p','a','c','e','-','p','o','i','n','t','e','r','.','l','o','c','a','l'
  };
  const char *personalization = "space-pointer-tls";
  const char *distinguishedName = "CN=space-pointer.local,O=Universal Space Pointer,C=IL";

  mbedtls_entropy_init(&entropy);
  mbedtls_ctr_drbg_init(&randomGenerator);
  mbedtls_pk_init(&privateKey);
  mbedtls_x509write_crt_init(&certificateWriter);

  result = mbedtls_ctr_drbg_seed(&randomGenerator, mbedtls_entropy_func,
                                 &entropy,
                                 reinterpret_cast<const unsigned char *>(personalization),
                                 strlen(personalization));
  if (result != 0) {
    Serial.printf("TLS RNG initialization failed: %d\n", result);
    goto cleanup;
  }

  result = mbedtls_pk_setup(&privateKey, mbedtls_pk_info_from_type(MBEDTLS_PK_RSA));
  if (result != 0) {
    Serial.printf("TLS RSA key setup failed: %d\n", result);
    goto cleanup;
  }

  result = mbedtls_rsa_gen_key(mbedtls_pk_rsa(privateKey),
                               mbedtls_ctr_drbg_random, &randomGenerator,
                               2048, 65537);
  if (result != 0) {
    Serial.printf("TLS RSA key generation failed: %d\n", result);
    goto cleanup;
  }

  // mbedTLS writes DER data at the end of its output buffer.
  temporaryKey = new (std::nothrow) unsigned char[4096];
  if (temporaryKey == nullptr) {
    result = -1;
    goto cleanup;
  }
  keyBytesWritten = mbedtls_pk_write_key_der(&privateKey, temporaryKey, 4096);
  if (keyBytesWritten <= 0) {
    result = keyBytesWritten;
    goto cleanup;
  }
  savedKey = new (std::nothrow) unsigned char[keyBytesWritten];
  if (savedKey == nullptr) {
    result = -1;
    goto cleanup;
  }
  memcpy(savedKey, temporaryKey + 4096 - keyBytesWritten, keyBytesWritten);
  delete[] temporaryKey;
  temporaryKey = nullptr;

  mbedtls_x509write_crt_set_version(&certificateWriter, MBEDTLS_X509_CRT_VERSION_3);
  mbedtls_x509write_crt_set_md_alg(&certificateWriter, MBEDTLS_MD_SHA256);
  mbedtls_x509write_crt_set_subject_key(&certificateWriter, &privateKey);
  mbedtls_x509write_crt_set_issuer_key(&certificateWriter, &privateKey);

  result = mbedtls_x509write_crt_set_subject_name(&certificateWriter, distinguishedName);
  if (result == 0) result = mbedtls_x509write_crt_set_issuer_name(&certificateWriter, distinguishedName);
  if (result == 0) result = mbedtls_x509write_crt_set_validity(&certificateWriter, "20260101000000", "20351231235959");
  if (result == 0) result = mbedtls_x509write_crt_set_basic_constraints(&certificateWriter, 1, 0);
  if (result == 0) {
    // Modern browsers check Subject Alternative Name, not just Common Name.
    result = mbedtls_x509write_crt_set_extension(
        &certificateWriter, MBEDTLS_OID_SUBJECT_ALT_NAME,
        MBEDTLS_OID_SIZE(MBEDTLS_OID_SUBJECT_ALT_NAME),
        0, subjectAltName, sizeof(subjectAltName));
  }

#if MBEDTLS_VERSION_NUMBER >= 0x03000000
  if (result == 0) {
    // The RSA key is unique per device; this serial identifies the local certificate.
    unsigned char serial[] = {0x53, 0x50, 0x32, 0x30, 0x32, 0x36};
    result = mbedtls_x509write_crt_set_serial_raw(&certificateWriter, serial, sizeof(serial));
  }
#else
  if (result == 0) result = mbedtls_mpi_fill_random(&serialNumber, 10, mbedtls_ctr_drbg_random, &randomGenerator);
  if (result == 0) result = mbedtls_x509write_crt_set_serial(&certificateWriter, &serialNumber);
#endif
  if (result != 0) goto cleanup;

  temporaryCertificate = new (std::nothrow) unsigned char[4096];
  if (temporaryCertificate == nullptr) {
    result = -1;
    goto cleanup;
  }
  certificateBytesWritten = mbedtls_x509write_crt_der(
      &certificateWriter, temporaryCertificate, 4096,
      mbedtls_ctr_drbg_random, &randomGenerator);
  if (certificateBytesWritten <= 0) {
    result = certificateBytesWritten;
    goto cleanup;
  }

  savedCertificate = new (std::nothrow) unsigned char[certificateBytesWritten];
  if (savedCertificate == nullptr) {
    result = -1;
    goto cleanup;
  }
  memcpy(savedCertificate, temporaryCertificate + 4096 - certificateBytesWritten,
         certificateBytesWritten);

  // SSLCert stores pointers rather than copying. Transfer these buffers to it;
  // they remain allocated for the lifetime of the HTTPS server.
  certContext.setPK(savedKey, keyBytesWritten);
  certContext.setCert(savedCertificate, certificateBytesWritten);
  savedKey = nullptr;
  savedCertificate = nullptr;
  result = 0;

cleanup:
  delete[] temporaryKey;
  delete[] savedKey;
  delete[] temporaryCertificate;
  delete[] savedCertificate;
#if MBEDTLS_VERSION_NUMBER < 0x03000000
  mbedtls_mpi_free(&serialNumber);
#endif
  mbedtls_x509write_crt_free(&certificateWriter);
  mbedtls_pk_free(&privateKey);
  mbedtls_ctr_drbg_free(&randomGenerator);
  mbedtls_entropy_free(&entropy);
  return result;
}

// Persist the self-signed certificate so it remains stable across reboots.
// This avoids generating a different browser identity each time the ESP32 starts.
bool configureHttpsCertificate() {
  Preferences preferences;
  if (!preferences.begin("space-tls", false)) {
    Serial.println("ERROR: Could not open NVS for TLS certificate.");
    return false;
  }

  const size_t certLength = preferences.getBytesLength("cert");
  const size_t keyLength = preferences.getBytesLength("key");
  bool loaded = false;

  // Validate stored lengths before allocating, in case NVS contains bad data.
  if (certLength > 0 && certLength <= 4096 && keyLength > 0 && keyLength <= 4096) {
    unsigned char *certBytes = new (std::nothrow) unsigned char[certLength];
    unsigned char *keyBytes = new (std::nothrow) unsigned char[keyLength];
    if (certBytes != nullptr && keyBytes != nullptr &&
        preferences.getBytes("cert", certBytes, certLength) == certLength &&
        preferences.getBytes("key", keyBytes, keyLength) == keyLength) {
      localTlsCertificate.setCert(certBytes, certLength);
      localTlsCertificate.setPK(keyBytes, keyLength);
      loaded = true;
      Serial.println("Loaded the existing local TLS certificate from NVS.");
    } else {
      delete[] certBytes;
      delete[] keyBytes;
    }
  }

  if (!loaded) {
    Serial.println("Generating a local self-signed TLS certificate (first boot may take a while)...");
    const int result = createSpacePointerCertificate(localTlsCertificate);
    if (result != 0) {
      Serial.printf("ERROR: TLS certificate generation failed (0x%02X).\n", result);
      preferences.end();
      return false;
    }

    const bool certSaved = preferences.putBytes("cert", localTlsCertificate.getCertData(),
                                                localTlsCertificate.getCertLength()) == localTlsCertificate.getCertLength();
    const bool keySaved = preferences.putBytes("key", localTlsCertificate.getPKData(),
                                               localTlsCertificate.getPKLength()) == localTlsCertificate.getPKLength();
    if (!certSaved || !keySaved) {
      Serial.println("ERROR: Could not persist TLS certificate; check NVS capacity.");
      preferences.end();
      return false;
    }
    Serial.println("Created and saved the local self-signed TLS certificate.");
  }

  preferences.end();
  server.setServerKeyAndCert(
      localTlsCertificate.getPKData(), localTlsCertificate.getPKLength(),
      localTlsCertificate.getCertData(), localTlsCertificate.getCertLength());
  return localTlsCertificate.getPKLength() > 0 && localTlsCertificate.getCertLength() > 0;
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

int pidOutput(const PController &p, float error) {
    // Invalid sensor/target math must never turn into a motor command.
    if (!isfinite(error)) return 0;

    // Ignore tiny errors so sensor noise does not make the mount chatter.
    if (fabsf(error) <= p.deadband) return 0;

    // Round to the nearest whole step rather than truncating toward zero.
    const long requestedSteps = lroundf(p.kp * error);

    // Keep a hard ceiling for abnormal errors; this is not a speed ramp.
    // Normal tracking corrections should be only a few steps per control tick.
    return (int)constrain((float)requestedSteps,
                          -(float)MAX_STEPS_PER_TICK,
                          (float)MAX_STEPS_PER_TICK);
}

// Calculate the next satellite pass or planetary rise/set independently of
// any display. The web API reads the result from the shared telemetry state.
void updateNextPass(const TargetState &target, time_t now,
                    double lat, double lon, double alt) {
    bool passValid = false;
    time_t aosTime = 0;
    time_t losTime = 0;
    double maxElevation = 0.0;

    if (target.type == TYPE_SATELLITE && tleReady &&
        xSemaphoreTake(sgp4Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
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

    if (xSemaphoreTake(sysMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        sys.nextPassValid = passValid;
        sys.nextPassAOS = aosTime;
        sys.nextPassLOS = losTime;
        sys.nextPassMaxEl = maxElevation;
        xSemaphoreGive(sysMutex);
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

      // Pass/rise/set predictions are telemetry too, so update them here
      // rather than tying their calculation to a physical display task.
      static uint32_t lastPassCalc = 0;
      if (millis() - lastPassCalc > 30000) {
          updateNextPass(target, now, lat, lon, alt);
          lastPassCalc = millis();
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

          bool motorTargetWritten = false;
          if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
              const bool stillSelected =
                  current.type == target.type &&
                  ((target.type == TYPE_SATELLITE && current.noradID == target.noradID) ||
                   (target.type == TYPE_PLANET && current.body == target.body));
              if (stillSelected) {
                  motorTarget.az = (float)targetAz;
                  motorTarget.el = (float)targetEl;
                  motorTargetWritten = true;
              }
              xSemaphoreGive(targetMutex);
          }
          targetCalculationReady = motorTargetWritten;
      } else {
          targetCalculationReady = false;
      }

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
      if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          activeTarget = current;
          targetAz = motorTarget.az;
          targetEl = motorTarget.el;
          targetRead = true;
          xSemaphoreGive(targetMutex);
      }

      const bool orbitalDataReady = activeTarget.type != TYPE_SATELLITE || tleReady;
      if (!targetRead || !timeReady || !observerReady || !targetCalculationReady ||
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

      if (!driversEnabled) {
          digitalWrite(AZ_EN_PIN, LOW);
          digitalWrite(EL_EN_PIN, LOW);
          driversEnabled = true;
      }

      // Convert the BNO055 magnetic heading to true north before comparing
      // it with Astronomy Engine's true-north target azimuth.
      float headingTrue = fmodf(headingNow + MAGNETIC_DECLINATION_DEG, 360.0f);
      if (headingTrue < 0.0f) headingTrue += 360.0f;

      const int azSteps = pidOutput(pidAz, wrapAngle(targetAz - headingTrue));
      const int elSteps = pidOutput(pidEl, wrapAngle(targetEl - elNow));
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
    // ipwho.is exposes a no-key JSON endpoint over HTTPS; IP-based location is approximate.
    if (!http.begin("https://ipwho.is/")) return false;
    const int status = http.GET();
    bool valid = false;
    if (status == HTTP_CODE_OK) {
        StaticJsonDocument<512> doc;
        const DeserializationError error = deserializeJson(doc, http.getString());
        if (!error && doc["success"] == true) {
            const double lat = doc["latitude"] | NAN;
            const double lon = doc["longitude"] | NAN;
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
                const char *rawLine1 = doc["line1"].as<const char *>();
                const char *rawLine2 = doc["line2"].as<const char *>();

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
                Serial.printf("ERROR: TLE server returned HTTP %d\n", status);
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