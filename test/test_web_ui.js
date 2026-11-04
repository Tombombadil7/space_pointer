const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const firmware = fs.readFileSync(
  path.join(__dirname, "../src/spacepointer_ver_2.cpp"),
  "utf8"
);

// Execute the exact JavaScript embedded in the dashboard with small browser mocks.
const header = fs.readFileSync(
  path.join(__dirname, "../include/web_ui.h"),
  "utf8"
);
const scriptMatch = header.match(/<script>([\s\S]*?)<\/script>/);
assert.ok(scriptMatch, "dashboard JavaScript must exist in web_ui.h");

function createHarness(fetchHandler) {
  const calls = [];
  const alerts = [];
  const elements = new Map();
  const labels = new Map();
  let pollCallback = null;
  let mapPan = null;
  let markerPosition = null;

  function element(id) {
    if (!elements.has(id)) {
      elements.set(id, { innerText: "", style: {} });
    }
    return elements.get(id);
  }

  const context = {
    console,
    URLSearchParams,
    alert: (message) => alerts.push(message),
    setInterval: (callback) => {
      pollCallback = callback;
      return 1;
    },
    fetch: (url, options) => {
      calls.push({ url, options });
      return fetchHandler(url, options);
    },
    navigator: {
      geolocation: {
        getCurrentPosition: (success, failure) => {
          context.lastGeoFailure = failure;
          success({
            coords: {
              latitude: 32.0853,
              longitude: 34.7818,
              altitude: 0
            }
          });
        }
      }
    },
    document: {
      getElementById: element,
      querySelector: (selector) => {
        if (!labels.has(selector)) labels.set(selector, { innerText: "" });
        return labels.get(selector);
      }
    },
    L: {
      map: () => ({
        setView: () => ({
          panTo: (position) => { mapPan = position; }
        })
      }),
      tileLayer: () => ({ addTo: () => {} }),
      marker: () => ({
        addTo: () => ({
          setLatLng: (position) => { markerPosition = position; }
        })
      })
    }
  };

  vm.runInNewContext(scriptMatch[1], context, { filename: "dashboard.js" });

  return {
    context,
    calls,
    alerts,
    element,
    label: (selector) => labels.get(selector),
    poll: () => {
      assert.equal(typeof pollCallback, "function", "poll interval was registered");
      pollCallback();
    },
    get mapPan() { return mapPan; },
    get markerPosition() { return markerPosition; }
  };
}

async function flushPromises() {
  await new Promise((resolve) => setImmediate(resolve));
}


function routeSource(path) {
  const start = firmware.indexOf(`server.on("${path}"`);
  assert.notEqual(start, -1, `route ${path} must exist`);
  const end = firmware.indexOf("\n  });", start);
  assert.notEqual(end, -1, `route ${path} must have a handler boundary`);
  return firmware.slice(start, end);
}

test("firmware mutation endpoints require POST, authentication, and CSRF header", () => {
  for (const endpoint of ["/set", "/updateLoc"]) {
    const route = routeSource(endpoint);
    assert.match(route, /HTTP_POST/);
    assert.match(route, /r->authenticate\(CONTROL_USER, CONTROL_PASSWORD\)/);
    assert.match(route, /hasValidMutationRequest\(r\)/);
    assert.ok(
      route.indexOf("r->authenticate") < route.indexOf("hasValidMutationRequest"),
      "authentication must be checked before the mutation header"
    );
  }
});

test("firmware telemetry endpoint remains read-only GET", () => {
  const route = routeSource("/data");
  assert.match(route, /HTTP_GET/);
  assert.match(route, /getTelemetrySnapshot\(target, data\)/);
  assert.match(route, /r->send\(503/);
  assert.match(route, /application\/json/);
});

test("target changes use POST, form encoding, and the CSRF header", async () => {
  const h = createHarness(async () => ({ ok: true, status: 202 }));
  h.context.setTarget(0, "ISS", 25544);
  await flushPromises();

  assert.equal(h.calls.length, 1);
  assert.equal(h.calls[0].url, "/set");
  assert.equal(h.calls[0].options.method, "POST");
  assert.equal(
    h.calls[0].options.headers["X-SpacePointer-Request"],
    "1"
  );
  assert.equal(
    h.calls[0].options.headers["Content-Type"],
    "application/x-www-form-urlencoded"
  );
  const body = new URLSearchParams(h.calls[0].options.body);
  assert.equal(body.get("type"), "0");
  assert.equal(body.get("name"), "ISS");
  assert.equal(body.get("val"), "25544");
});

test("phone GPS sync sends coordinates through the same POST helper", async () => {
  const h = createHarness(async () => ({ ok: true, status: 202 }));
  h.context.syncGPS();
  await flushPromises();

  assert.equal(h.calls.length, 1);
  assert.equal(h.calls[0].url, "/updateLoc");
  assert.equal(h.calls[0].options.method, "POST");
  const body = new URLSearchParams(h.calls[0].options.body);
  assert.equal(body.get("lat"), "32.0853");
  assert.equal(body.get("lon"), "34.7818");
  // The dashboard deliberately falls back when the phone reports null/zero altitude.
  assert.equal(body.get("alt"), "50");
  assert.equal(h.alerts.at(-1), "GPS Updated!");
});

test("failed target mutation is surfaced to the user", async () => {
  const h = createHarness(async () => ({ ok: false, status: 403 }));
  h.context.setTarget(1, "Mars", 3);
  await flushPromises();

  assert.deepEqual(h.alerts, ["Target update failed: 403"]);
});

test("telemetry HTTP errors replace only the telemetry section", async () => {
  const h = createHarness(async () => ({ ok: false, status: 503 }));
  h.poll();
  await flushPromises();

  assert.equal(h.element("telemetry-content").style.display, "none");
  assert.equal(h.element("telemetry-error").style.display, "block");
  // Map and target/location controls are outside the telemetry container.
  assert.ok(h.element("map"));
  assert.ok(h.element("norad"));
});

test("telemetry recovers after a temporary API failure", async () => {
  let fail = true;
  const h = createHarness(async () => {
    if (fail) return { ok: false, status: 503 };
    return {
      ok: true,
      json: async () => ({
        type: 0, name: "ISS", az: 359.5, el: 45.0,
        sLat: 12.5, sLon: 34.5, speed: 7.66, alt: 420,
        dist: 500, nextPass: "12:30 UTC", nextPassLOS: "12:40 UTC",
        nextPassEl: 55
      })
    };
  });

  h.poll();
  await flushPromises();
  assert.equal(h.element("telemetry-content").style.display, "none");

  fail = false;
  h.poll();
  await flushPromises();

  assert.equal(h.element("telemetry-error").style.display, "none");
  assert.equal(h.element("telemetry-content").style.display, "block");
  assert.equal(h.element("target-name").innerText, "ISS");
  assert.equal(h.element("az").innerText, "359.50");
  assert.equal(h.element("speed").innerText, "7.66");
  assert.equal(h.markerPosition[0], 12.5);
  assert.equal(h.markerPosition[1], 34.5);
  assert.equal(h.mapPan[0], 12.5);
  assert.equal(h.mapPan[1], 34.5);
});

test("planet telemetry displays speed as unavailable, not zero", async () => {
  const h = createHarness(async () => ({
    ok: true,
    json: async () => ({
      type: 1, name: "Mars", az: 120, el: 30,
      sLat: 0, sLon: 0, speed: 0, alt: 200000000,
      dist: 200000000, nextPass: "06:00 UTC",
      nextPassLOS: "18:00 UTC", nextPassEl: 0
    })
  }));

  h.poll();
  await flushPromises();

  assert.equal(h.element("speed").innerText, "—");
  assert.equal(h.label('[data-lbl="speed"]').innerText, "not calculated");
  assert.equal(h.label('[data-lbl="pass"]').innerText, "Rises at");
  assert.equal(h.label('[data-lbl="lose"]').innerText, "Sets at");
});

test("firmware system state uses one mutex and one configuration generation", () => {
  assert.ok(firmware.includes("SemaphoreHandle_t stateMutex;"));
  for (const oldName of [
    "targetMutex", "sysMutex", "targetGeneration", "observerGeneration",
    "GenerationReadiness", "telemetrySourceVersions", "passSourceVersions",
    "calculationReadiness", "tleSourceVersions"
  ]) {
    assert.equal(firmware.includes(oldName), false, oldName + " should be removed");
  }
  assert.ok(firmware.includes("systemReadiness.invalidate();"));
  assert.ok(routeSource("/set").includes("advanceConfigurationLocked();"));
  assert.ok(routeSource("/updateLoc").includes("advanceConfigurationLocked();"));
});

test("motor stop requests use a latched task notification, not a shared bool", () => {
  const stopStart = firmware.indexOf("void disableMotors() {");
  const stopEnd = firmware.indexOf("bool startTLEFetchTask()", stopStart);
  assert.notEqual(stopStart, -1);
  assert.notEqual(stopEnd, -1);
  const disableFunction = firmware.slice(stopStart, stopEnd);
  assert.ok(disableFunction.includes(
    "xTaskNotify(motorTaskHandle, MOTOR_STOP_NOTIFICATION, eSetBits)"
  ));

  const motorStart = firmware.indexOf("void MotorTask(void * p) {");
  const motorEnd = firmware.indexOf("void stepMotor(", motorStart);
  assert.notEqual(motorStart, -1);
  assert.notEqual(motorEnd, -1);
  const motorTask = firmware.slice(motorStart, motorEnd);
  assert.ok(motorTask.includes(
    "xTaskNotifyWait(0, MOTOR_STOP_NOTIFICATION"
  ));
  assert.equal(firmware.includes("motorStopRequested"), false);
});
