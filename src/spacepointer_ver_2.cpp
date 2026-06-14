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
#define AZ_EN_PIN      25    // LOW = enabled; pull LOW permanently if unused

// Elevation A4988
#define EL_STEP_PIN    14
#define EL_DIR_PIN     15
#define EL_EN_PIN      26    // LOW = enabled; pull LOW permanently if unused
// MS pins: tie physically to 3.3V for 1/16 microstepping (no code needed)
// A4988: MS1=HIGH, MS2=HIGH, MS3=HIGH → 1/16 step

// ── Motor constants ─────────────────────────────────────────────────────
// 28BYJ-48 bipolar: 2048 full steps/rev × 16 microsteps = 32768 microsteps/rev
#define MICROSTEPS_PER_REV  32768UL
#define STEP_PULSE_US     50      // µs per pulse
#define MOTOR_TASK_MS     25      // MotorTask period in ms
#define MAX_STEPS_PER_TICK  ((MOTOR_TASK_MS * 1000) / (STEP_PULSE_US * 2 * 2))
// = (25000) / (200) = 125 steps max — leaves a small margin for overhead

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

PID pidAz = {2.5f, 0.02f, 0.4f, 1.5f};   // tune these after first run
PID pidEl = {3.0f, 0.03f, 0.5f, 1.0f};
SemaphoreHandle_t targetMutex;
SemaphoreHandle_t i2cMutex;
// ── AS5600 encoder ──────────────────────────────────────────────────────
AS5600 encoder;   // default I2C address 0x36, shares bus with BNO055 (0x28)
// LCD — try 0x27 first, some boards use 0x3F
// Change to LiquidCrystal_I2C lcd(0x3F, 16, 2) if 0x27 doesn't work
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool lcdFound = false;   // set in setup(), checked before creating DisplayTask


void DisplayTask(void * p);
void updateIPLocation();
void fetchTLE();
void PhysicsTask(void * p);
void stepMotor(int stepPin, int dirPin, bool forward);
int pidOutput(PID &p, float error, float dt);
float wrapAngle(float e);
void MotorTask(void * p);


const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <title>Universal Space Pointer</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
    <style>
      body { font-family: 'Segoe UI', Arial; text-align: center; background: #121212; color: white; margin: 0; padding: 15px; }
      #map { height: 320px; width: 95%; margin: 15px auto; border-radius: 12px; border: 2px solid #444; }
      .card { background: #1e1e1e; padding: 20px; border-radius: 15px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); max-width: 500px; margin: auto; }
      .btn { padding: 10px 15px; margin: 5px; cursor: pointer; border-radius: 6px; border: none; background: #3498db; color: white; transition: 0.2s; }
      .btn:hover { background: #2980b9; }
      .input-group { margin: 20px 0; display: flex; justify-content: center; gap: 10px; flex-wrap: wrap; }
      select, input { padding: 8px; border-radius: 5px; border: none; background: #333; color: white; }
      .data { color: #00ffcc; font-weight: bold; }
      .label { color: #888; font-size: 0.85em; }
  
      /* ── tracking name banner ── */
      #target-name {
        font-size: 1.6em; font-weight: bold; letter-spacing: 2px;
        color: #00ffcc; margin: 0 0 12px; text-transform: uppercase;
      }
  
      /* ── stats grid ── */
      .stats-grid {
        display: grid; grid-template-columns: repeat(3, 1fr);
        gap: 10px; margin: 12px 0;
      }
      .stat-box {
        background: #2a2a2a; border-radius: 10px; padding: 10px 6px;
        border: 1px solid #333;
      }
      .stat-val { font-size: 1.2em; color: #00ffcc; font-weight: bold; }
      .stat-lbl { font-size: 0.72em; color: #888; margin-top: 3px; }
  
      /* ── next pass strip ── */
      #next-pass-bar {
        background: #2a2a2a; border-radius: 10px; padding: 10px 14px;
        margin: 10px 0; font-size: 0.9em; border: 1px solid #333;
        display: flex; justify-content: space-between; align-items: center;
      }
      #next-pass-bar .label { font-size: 0.8em; }
    </style>
  </head>
  <body>
    <div class="card">
  
      <!-- tracking target name -->
      <p id="target-name">---</p>
  
      <div id="map"></div>
  
      <!-- az / el row -->
      <p>
        <span class="label">Az</span> <span id="az" class="data">0</span>&deg;&nbsp;&nbsp;
        <span class="label">El</span> <span id="el" class="data">0</span>&deg;
      </p>
  
      <!-- speed / altitude / distance grid -->
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
  
      <!-- next visible pass / rise+set -->
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
        <button class="btn" onclick="setTarget(1, 'Moon', 0)">Moon</button>
        <button class="btn" onclick="setTarget(1, 'Mars', 1)">Mars</button>
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
  
      function setTarget(type, name, val) { fetch('/set?type='+type+'&name='+name+'&val='+val); }
      function setCustomSat() {
        var id = document.getElementById('norad').value;
        if(id) setTarget(0, 'Custom', id);
      }
      function syncGPS() {
        navigator.geolocation.getCurrentPosition(function(p) {
          fetch('/updateLoc?lat='+p.coords.latitude+'&lon='+p.coords.longitude+'&alt='+(p.coords.altitude||50));
          alert("GPS Updated!");
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
          document.querySelector('[data-lbl="alt"]').innerText  = isSat ? 'km altitude'   : 'km from Earth';
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
          marker.setLatLng([d.sLat, d.sLon]);
          if (isSat) map.panTo([d.sLat, d.sLon]);
        });
      }, 2000);
    </script>
  </body>
  </html>)rawliteral";


void setup() {
  // ── 1. System Initialization ───────────────────────────────────────
  Serial.begin(115200);
  Serial.println("\nStarting Universal Space Pointer...");

  // ── 2. FreeRTOS Primitives ─────────────────────────────────────────
  targetMutex = xSemaphoreCreateMutex();
  i2cMutex    = xSemaphoreCreateMutex();
  if (targetMutex == NULL || i2cMutex == NULL) {
      Serial.println("CRITICAL: Failed to create mutexes! Halting.");
      while(1);
  }

  // ── 3. Hardware Initialization ─────────────────────────────────────
  pinMode(AZ_STEP_PIN, OUTPUT); pinMode(AZ_DIR_PIN, OUTPUT);
  pinMode(EL_STEP_PIN, OUTPUT); pinMode(EL_DIR_PIN, OUTPUT);
  pinMode(AZ_EN_PIN, OUTPUT);   digitalWrite(AZ_EN_PIN, LOW);
  pinMode(EL_EN_PIN, OUTPUT);   digitalWrite(EL_EN_PIN, LOW);

  // I2C bus — shared by BNO055, AS5600, and LCD
  Wire.begin(); // SDA=21, SCL=22
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
  if (!encoder.isConnected()) {
      Serial.println("WARNING: AS5600 not found — check wiring");
  }
  encoder.setDirection(AS5600_CLOCK_WISE);

  // ── BNO055 IMU ─────────────────────────────────────────────────────
  if (!bno.begin()) {
      Serial.println("WARNING: BNO055 not found — check wiring");
  }

  // ── 4. Network & Web Services ──────────────────────────────────────
  WiFiManager wm;
  wm.autoConnect("SpaceTracker_AP");
  updateIPLocation();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
      r->send(200, "text/html", index_html);
  });

  server.on("/updateLoc", HTTP_GET, [](AsyncWebServerRequest *r){
      sys.lat = r->arg("lat").toDouble();
      sys.lon = r->arg("lon").toDouble();
      sys.alt = r->arg("alt").toDouble();
      if (current.type == TYPE_SATELLITE)
          xTaskCreate([](void*){ fetchTLE(); vTaskDelete(NULL); }, "TLE", 8000, NULL, 1, NULL);
      r->send(200, "text/plain", "OK");
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *r){
      int    type = r->arg("type").toInt();
      String name = r->arg("name");
      long   val  = r->arg("val").toInt();
      if (type == 0) {
          current.type    = TYPE_SATELLITE;
          current.noradID = val;
          strncpy(current.name, name.c_str(), 20);
          xTaskCreate([](void*){ fetchTLE(); vTaskDelete(NULL); }, "TLE", 8000, NULL, 1, NULL);
      } else {
          current.type = TYPE_PLANET;
          current.body = (astro_body_t)val;
          strncpy(current.name, name.c_str(), 20);
      }
      r->send(200, "text/plain", "OK");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *r){
      StaticJsonDocument<512> doc;
      doc["name"]  = current.name;
      doc["type"]  = current.type;
      doc["az"]    = sys.targetAz;
      doc["el"]    = sys.targetEl;
      doc["sLat"]  = sys.satLat;
      doc["sLon"]  = sys.satLon;
      doc["speed"] = sys.satSpeed;
      doc["alt"]   = satSgp4.satAlt;
      doc["dist"]  = satSgp4.satDist;
      if (sys.nextPassValid) {
          struct tm *aos = gmtime(&sys.nextPassAOS);
          char buf[20];
          snprintf(buf, sizeof(buf), "%02d:%02d UTC", aos->tm_hour, aos->tm_min);
          doc["nextPass"]   = buf;
          doc["nextPassEl"] = (int)sys.nextPassMaxEl;
      } else {
          doc["nextPass"]   = "--:--";
          doc["nextPassEl"] = 0;
      }
      String out; serializeJson(doc, out);
      r->send(200, "application/json", out);
  });

  server.begin();

  // ── 5. Time Synchronization ────────────────────────────────────────
  configTime(7200, 3600, "pool.ntp.org");
  Serial.print("Waiting for NTP time sync...");
  time_t now;
  while (time(&now) < 1000000000) { delay(100); Serial.print("."); }
  Serial.println("\nTime synchronized!");

  if (lcdFound) {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("WiFi OK");
      lcd.setCursor(0, 1); lcd.print("Time synced!");
      delay(1000);
  }

  // ── 6. Task Creation ───────────────────────────────────────────────
  // WiFi stack runs on Core 0 — put Physics on Core 1 to avoid contention
  xTaskCreatePinnedToCore(PhysicsTask, "Physics", 15000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(MotorTask,   "Motors",  8000,  NULL, 3, NULL, 0);

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


float wrapAngle(float e) {          // keep error in -180..+180
  while (e >  180.f) e -= 360.f;
  while (e < -180.f) e += 360.f;
  return e;
}

int pidOutput(PID &p, float error, float dt) {
    if (fabsf(error) < p.deadband) { p.integral = 0; return 0; }
    p.integral  = constrain(p.integral + error * dt, -40.f, 40.f);
    float deriv = (error - p.lastErr) / dt;
    p.lastErr   = error;
    float out   = p.kp * error + p.ki * p.integral + p.kd * deriv;
    return (int)constrain(out, -(float)MAX_STEPS_PER_TICK, (float)MAX_STEPS_PER_TICK);
}

void PhysicsTask(void * p) {

  TickType_t lastWake = xTaskGetTickCount();
  const float DT = 0.5f;   // seconds — matches vTaskDelay below

  for(;;) {
      time_t now; time(&now);
      struct tm * t = gmtime(&now);


        if (current.type == TYPE_SATELLITE) {
            // ── 1. Primary position ──────────────────────────────
            satSgp4.findsat((unsigned long)now);
            sys.targetAz = satSgp4.satAz;
            sys.targetEl = satSgp4.satEl;
            sys.satLat   = satSgp4.satLat;
            sys.satLon   = satSgp4.satLon;

            // ── 2. Speed: position 1s later, then rewind ─────────
            double lat1 = satSgp4.satLat, lon1 = satSgp4.satLon, alt1 = satSgp4.satAlt;
            satSgp4.findsat((unsigned long)(now + 1));
            double dlat = (satSgp4.satLat - lat1) * DEG_TO_RAD * 6371.0;
            double dlon = (satSgp4.satLon - lon1) * DEG_TO_RAD * 6371.0
                          * cos(lat1 * DEG_TO_RAD);
            double dalt = satSgp4.satAlt - alt1;
            sys.satSpeed = sqrt(dlat*dlat + dlon*dlon + dalt*dalt); // km/s

          } else {
            // ── position at t ──────────────────────────────────────────
            astro_time_t aTime = Astronomy_MakeTime(
                t->tm_year+1900, t->tm_mon+1, t->tm_mday,
                t->tm_hour, t->tm_min, (double)t->tm_sec);
            astro_observer_t obs = { sys.lat, sys.lon, sys.alt };
        
            // distance from Earth center (AU) → km
            astro_equatorial_t equ1 = Astronomy_Equator((astro_body_t)current.body, &aTime, obs, EQUATOR_OF_DATE, ABERRATION);
            double distAU = equ1.dist;                        // AU
            sys.satAltKm  = distAU * 149597870.7;             // km from Earth center
            sys.satDist   = sys.satAltKm;                     // reuse same field for /data
        
            astro_horizon_t hor1 = Astronomy_Horizon(&aTime, obs, equ1.ra, equ1.dec, REFRACTION_NORMAL);
            sys.targetAz = hor1.azimuth;
            sys.targetEl = hor1.altitude;
        
            // ── position at t+1s ──────────────────────────────────────
            astro_time_t aTime2 = Astronomy_MakeTime(
                t->tm_year+1900, t->tm_mon+1, t->tm_mday,
                t->tm_hour, t->tm_min, (double)t->tm_sec + 1.0/86400.0);
            astro_equatorial_t equ2 = Astronomy_Equator((astro_body_t)current.body, &aTime2, obs, EQUATOR_OF_DATE, ABERRATION);
            double distAU2 = equ2.dist;
            double dDist   = (distAU2 - distAU) * 149597870.7; // km radial
            astro_horizon_t hor2 = Astronomy_Horizon(&aTime2, obs, equ2.ra, equ2.dec, REFRACTION_NORMAL);
        
            // angular movement (degrees) → arc length (km)
            double dAz  = (hor2.azimuth - hor1.azimuth) * DEG_TO_RAD * sys.satAltKm;
            double dEl  = (hor2.altitude - hor1.altitude) * DEG_TO_RAD * sys.satAltKm;
            sys.satSpeed = sqrt(dAz*dAz + dEl*dEl + dDist*dDist); // km/s
        
            sys.satLat = 0; sys.satLon = 0;
        }

        if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          motorTarget.az = (float)sys.targetAz;
          motorTarget.el = (float)sys.targetEl;
          xSemaphoreGive(targetMutex);
      }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
    }
}


void MotorTask(void * p) {
  const float DT = 0.025f;
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
      float targetAz, targetEl;
      if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          targetAz = motorTarget.az;
          targetEl = motorTarget.el;
          xSemaphoreGive(targetMutex);
      }
      sensors_event_t event;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          bno.getEvent(&event);
          xSemaphoreGive(i2cMutex);
      }
      float headingNow = event.orientation.x;
      float elNow = 0;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          elNow = encoder.getRawAngle() * (360.f / 4096.f);
          xSemaphoreGive(i2cMutex);
      }
      if (elNow > 180.f) elNow -= 360.f;

      int azSteps = pidOutput(pidAz, wrapAngle(targetAz - headingNow), DT);
      int elSteps = pidOutput(pidEl, wrapAngle(targetEl - elNow), DT);
      bool azFwd = azSteps > 0, elFwd = elSteps > 0;
      int azN = abs(azSteps), elN = abs(elSteps);
      int maxN = max(azN, elN);
      for (int i = 0; i < maxN; i++) {
          if (i < azN) stepMotor(AZ_STEP_PIN, AZ_DIR_PIN, azFwd);
          if (i < elN) stepMotor(EL_STEP_PIN, EL_DIR_PIN, elFwd);
      }
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(25));
  }
}


void DisplayTask(void * p) {
  static uint32_t lastPassCalc = 0;
  static uint8_t  screen       = 0;   // alternates 0 / 1 every 2s
  char line0[17], line1[17];

  for (;;) {

      // ── Next pass / rise+set: recalculate every 30s ───────────────
      if (millis() - lastPassCalc > 30000) {
          time_t now; time(&now);

          if (current.type == TYPE_SATELLITE) {
              satSgp4.initpredpoint((unsigned long)now, 0.0);
              passinfo overpass;
              sys.nextPassValid = satSgp4.nextpass(&overpass, 20);
              if (sys.nextPassValid) {
                  sys.nextPassAOS   = (time_t)((overpass.jdstart - 2440587.5) * 86400.0);
                  sys.nextPassLOS   = (time_t)((overpass.jdstop  - 2440587.5) * 86400.0);
                  sys.nextPassMaxEl = overpass.maxelevation;
              }
          } else {
              struct tm *utc = gmtime(&now);
              astro_time_t aTime = Astronomy_MakeTime(
                  utc->tm_year+1900, utc->tm_mon+1, utc->tm_mday,
                  utc->tm_hour, utc->tm_min, (double)utc->tm_sec);
              astro_observer_t obs = { sys.lat, sys.lon, sys.alt };
              astro_search_result_t rise = Astronomy_SearchRiseSet(
                  (astro_body_t)current.body, obs, DIRECTION_RISE, aTime, 1.0);
              astro_search_result_t set  = Astronomy_SearchRiseSet(
                  (astro_body_t)current.body, obs, DIRECTION_SET,  aTime, 1.0);
              if (rise.status == ASTRO_SUCCESS && set.status == ASTRO_SUCCESS) {
                  sys.nextPassAOS   = (time_t)(rise.time.ut * 86400.0 + 946727935.816);
                  sys.nextPassLOS   = (time_t)(set.time.ut  * 86400.0 + 946727935.816);
                  sys.nextPassMaxEl = 0;
                  sys.nextPassValid = true;
              } else {
                  sys.nextPassValid = false;
              }
          }
          lastPassCalc = millis();
      }

      // ── Build lines based on current screen ───────────────────────
      if (screen == 0) {
          // ── Screen 0: Pointing data ───────────────────────────────
          // Row 0: "ISS  Az:247.3   "
          snprintf(line0, sizeof(line0), "%-4.4s Az:%-6.1f",
                   current.name, sys.targetAz);

          // Row 1: elevation + altitude, or next pass if below horizon
          if (sys.targetEl >= 0) {
              snprintf(line1, sizeof(line1), "El:%-5.1f %5.0fkm",
                       sys.targetEl, sys.satAltKm);
          } else {
              if (sys.nextPassValid) {
                  struct tm *aos = gmtime(&sys.nextPassAOS);
                  snprintf(line1, sizeof(line1), "El:%-4.1f >%02d:%02dUTC",
                           sys.targetEl, aos->tm_hour, aos->tm_min);
              } else {
                  snprintf(line1, sizeof(line1), "El:%-5.1f No pass",
                           sys.targetEl);
              }
          }

      } else {
          // ── Screen 1: Telemetry data ──────────────────────────────
          // Row 0: "ISS  7.66 km/s  "
          snprintf(line0, sizeof(line0), "%-4.4s %6.2fkm/s",
                   current.name, sys.satSpeed);

          // Row 1: distance — satellites in km, planets in scientific-friendly units
          if (current.type == TYPE_SATELLITE) {
              // e.g. "Dist:    421394km"  — always < 999999km for LEO/MEO
              snprintf(line1, sizeof(line1), "Dist:%9.0fkm",
                       sys.satDist);
          } else {
              // Planets are millions of km — show in M km
              // e.g. "Dist:   384400Mkm"  (Moon ~384k km, Mars ~225M km)
              double distMkm = sys.satDist / 1000.0;
              if (distMkm < 10000.0) {
                  snprintf(line1, sizeof(line1), "Dist:%7.0f Mkm",
                           distMkm);
              } else {
                  // Very far (outer planets) — show in AU  e.g. "Dist:      5.2 AU"
                  snprintf(line1, sizeof(line1), "Dist:    %5.2f AU",
                           sys.satDist / 149597870.7);
              }
          }
      }

      // ── Write to LCD (i2cMutex guards shared I2C bus) ────────────
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          lcd.setCursor(0, 0); lcd.print(line0);
          lcd.setCursor(0, 1); lcd.print(line1);
          xSemaphoreGive(i2cMutex);
      }

      screen = 1 - screen;   // flip 0→1→0→1...
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

void updateIPLocation() {
    /* מבצע איכון ראשוני לפי כתובת IP כברירת מחדל  */
    HTTPClient http;
    http.begin("http://ip-api.com/json/");
    if (http.GET() == 200) {
        StaticJsonDocument<512> doc;
        deserializeJson(doc, http.getString());
        sys.lat = doc["lat"]; sys.lon = doc["lon"]; sys.alt = 50.0;
    }
    http.end();
}

void fetchTLE() {
    /* מושך נתוני מסלול מעודכנים מהאינטרנט [cite: 75, 76] */
    HTTPClient http;
    http.begin("https://tle.ivanstanojevic.me/api/tle/" + String(current.noradID));
    if(http.GET() == 200) {
        StaticJsonDocument<1024> doc;
        deserializeJson(doc, http.getString());
        satSgp4.site(sys.lat, sys.lon, sys.alt);
        char line1[130], line2[130];
        strncpy(line1, doc["line1"].as<const char*>(), sizeof(line1));
        strncpy(line2, doc["line2"].as<const char*>(), sizeof(line2));
        satSgp4.init(current.name, line1, line2);
    }
    http.end();
}