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
#include <FastAccelStepper.h>
#include <Adafruit_BNO055.h>
#include <Sgp4.h>
#include <astronomy.h>

// --- הגדרות חומרה (פינים) ---
#define AZ_STEP_PIN 12
#define AZ_DIR_PIN  13
#define EL_STEP_PIN 14
#define EL_DIR_PIN  15

// --- אובייקטים גלובליים ---
AsyncWebServer server(80);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperAz = NULL, *stepperEl = NULL;
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

void DisplayTask(void * p);
void updateIPLocation();
void fetchTLE();
void PhysicsTask(void * p);


// --- ממשק משתמש HTML (מאוחסן בזיכרון הפלאש) ---
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
        satSgp4.init(current.name, doc["line1"].as<const char*>(), doc["line2"].as<const char*>());
    }
    http.end();
}

void setup() {
    Serial.begin(115200);
    xTaskCreatePinnedToCore(DisplayTask, "Display", 8000, NULL, 1, NULL, 1); // Core 1 for display updates and next pass calculations
    WiFiManager wm;
    wm.autoConnect("SpaceTracker_AP");

    updateIPLocation(); // איכון ראשוני אוטומטי

    // אתחול מנועים (ליבה 1 מנהלת אותם ברקע) 
    engine.init();
    stepperAz = engine.stepperConnectToPin(AZ_STEP_PIN);
    stepperEl = engine.stepperConnectToPin(EL_STEP_PIN);
    if(stepperAz) { stepperAz->setDirectionPin(AZ_DIR_PIN); stepperAz->setAcceleration(8000); stepperAz->setSpeedInHz(4000); }
    if(stepperEl) { stepperEl->setDirectionPin(EL_DIR_PIN); stepperEl->setAcceleration(8000); stepperEl->setSpeedInHz(4000); }
    
    bno.begin(); // אתחול חיישן הכוון 
    // הגדרת כתובות שרת (Endpoints)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", index_html); });
    
    server.on("/updateLoc", HTTP_GET, [](AsyncWebServerRequest *r){
        sys.lat = r->arg("lat").toDouble(); sys.lon = r->arg("lon").toDouble(); sys.alt = r->arg("alt").toDouble();
        if(current.type == TYPE_SATELLITE) fetchTLE(); // רענון חישוב לווין לפי מיקום חדש
        r->send(200, "text/plain", "OK");
    });

    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *r){
      int type = r->arg("type").toInt();
      String name = r->arg("name");
      long val = r->arg("val").toInt();
      
      if(type == 0) { 
          current.type = TYPE_SATELLITE;
          current.noradID = val;
          strncpy(current.name, name.c_str(), 20); 
          fetchTLE(); 
      } else { 
          current.type = TYPE_PLANET;
          current.body = (astro_body_t)val;
          strncpy(current.name, name.c_str(), 20); 
      }
      r->send(200, "text/plain", "OK");
  });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *r){
      StaticJsonDocument<512> doc;  // bump from 256 — more fields now
      doc["name"]  = current.name;
      doc["type"]  = current.type;
      doc["az"]    = sys.targetAz;
      doc["el"]    = sys.targetEl;
      doc["sLat"]  = sys.satLat;
      doc["sLon"]  = sys.satLon;
      doc["speed"] = sys.satSpeed;
      doc["alt"]   = satSgp4.satAlt;   // km above Earth
      doc["dist"]  = satSgp4.satDist;  // km from observer
  
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
    configTime(7200, 3600, "pool.ntp.org"); // סנכרון זמן מדויק 
    
    // הרצת משימת הפיזיקה על ליבה 0 כדי לא להפריע למנועים 
    xTaskCreatePinnedToCore(PhysicsTask, "Physics", 15000, NULL, 1, NULL, 0);
}

void PhysicsTask(void * p) {
    for(;;) {
        time_t now; time(&now);
        struct tm * t = gmtime(&now);

        if (current.type == TYPE_SATELLITE) {
            // ── 1. Primary position ──────────────────────────────
            satSgp4.findsat(t->tm_year+1900, t->tm_mon+1, t->tm_mday,
                                    t->tm_hour, t->tm_min, t->tm_sec);
            sys.targetAz = satSgp4.satAz;
            sys.targetEl = satSgp4.satEl;
            sys.satLat   = satSgp4.satLat;
            sys.satLon   = satSgp4.satLon;

            // ── 2. Speed: position 1s later, then rewind ─────────
            double lat1 = satSgp4.satLat, lon1 = satSgp4.satLon, alt1 = satSgp4.satAlt;
            satSgp4.findsat(t->tm_year+1900, t->tm_mon+1, t->tm_mday,
                                    t->tm_hour, t->tm_min, t->tm_sec + 1);
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

        // ── 3. IMU + motors ──────────────────────────────────────
        sensors_event_t event; bno.getEvent(&event);
        double heading = event.orientation.x;
        double tilt    = event.orientation.y;
        stepperAz->moveTo((long)((sys.targetAz - heading) * 1422.22));
        stepperEl->moveTo((long)((sys.targetEl - tilt)    * 1422.22));

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void loop() {} // ריק - הכל מנוהל ב-Tasks

void DisplayTask(void * p) {
  // Initialize your display here (e.g. tft.init())
  static uint32_t lastPassCalc = 0;

  for(;;) {

      // ── Next pass / rise+set: expensive, run every 30s ───────
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
              // planet rise/set via Astronomy library
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
                  // J2000.0 UT days → Unix timestamp (946727935 = unix time of J2000.0)
                  sys.nextPassAOS   = (time_t)(rise.time.ut * 86400.0 + 946727935.816);
                  sys.nextPassLOS   = (time_t)(set.time.ut  * 86400.0 + 946727935.816);
                  sys.nextPassMaxEl = 0;   // not applicable for planets
                  sys.nextPassValid = true;
              } else {
                  sys.nextPassValid = false;  // circumpolar or always below horizon
              }
          }

          lastPassCalc = millis();
      }

      // ── Draw to display ──────────────────────────────────────
      // tft.fillScreen(TFT_BLACK);
      // tft.setCursor(0, 0);
      // tft.printf(">> %s\n", current.name);
      // tft.printf("Az: %.1f  El: %.1f\n", sys.targetAz, sys.targetEl);
      // tft.printf("Speed: %.2f km/s\n",   sys.satSpeed);
      // tft.printf("Alt:   %.0f km\n",      sys.satAltKm);
      // tft.printf("Dist:  %.0f km\n",      sys.satDist);
      // if (sys.nextPassValid) {
      //     struct tm *aos = gmtime(&sys.nextPassAOS);
      //     struct tm *los = gmtime(&sys.nextPassLOS);
      //     if (current.type == TYPE_SATELLITE) {
      //         tft.printf("Pass:  %02d:%02d UTC (max %.0f deg)\n",
      //                    aos->tm_hour, aos->tm_min, sys.nextPassMaxEl);
      //     } else {
      //         tft.printf("Rise:  %02d:%02d UTC\n", aos->tm_hour, aos->tm_min);
      //         tft.printf("Set:   %02d:%02d UTC\n", los->tm_hour, los->tm_min);
      //     }
      // }

      vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}