#pragma once

/*
 * Browser dashboard served by the ESP32 web server.
 *
 * Keeping the HTML/CSS/JavaScript in its own file makes the firmware logic
 * easier to navigate without changing the page contents or its behavior.
 */
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
    
        <div id="map"></div>

        <!-- Only the telemetry section is replaced when its API snapshot is unavailable.
             The map and all target/location controls remain visible and usable. -->
        <div id="telemetry-error" role="status" aria-live="polite"
             style="display:none; color:#ff7675; padding:18px 8px;">
          Telemetry temporarily unavailable. Please retry.
        </div>
        <div id="telemetry-content">
          <p id="target-name">---</p>

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
          fetch('/data').then(function(res) {
            // /data uses HTTP 503 only when the telemetry lock cannot be read.
            // Show the error inside the telemetry area, not over the whole page.
            if (!res.ok) throw new Error('Telemetry unavailable');
            return res.json();
          }).then(function(d) {
            document.getElementById('telemetry-error').style.display = 'none';
            document.getElementById('telemetry-content').style.display = 'block';
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
          }).catch(function() {
            // Keep the map and controls visible; replace only telemetry values.
            document.getElementById('telemetry-content').style.display = 'none';
            document.getElementById('telemetry-error').style.display = 'block';
          });
        }, 2000);
      </script>
    </body>
  </html>
  )rawliteral";
