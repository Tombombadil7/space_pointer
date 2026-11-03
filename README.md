# Universal Space Pointer

An experimental ESP32-S3 project for pointing a two-axis motorized mount at selected satellites and solar-system objects. The firmware combines orbital/astronomical calculations with an IMU, a magnetic rotary encoder, stepper-motor drivers, and a browser-based control panel.

> **Project status:** prototype / work in progress. The `fix/satellite-nextpass-minimum-elevation` branch contains the current software remediation pass and automated build/test workflows. The firmware has not yet been verified on the physical mount. Do not connect motor power until the pin map, motor wiring, current limits, mechanics, and emergency-stop arrangement have been checked.

## Intended features

- Track satellites by NORAD catalog ID using downloaded TLE data and the SGP4 library.
- Calculate pointing directions for the Moon, Mars, Jupiter, and Saturn using the bundled Astronomy Engine library.
- Drive azimuth and elevation stepper motors with A4988 drivers.
- Use a BNO055 IMU for heading feedback and an AS5600 magnetic encoder for elevation feedback.
- Display pointing and telemetry information through the browser-based web interface.
- Provide a local web interface with a satellite map, target selection, telemetry, and a phone-GPS location update.
- Estimate the observer's initial location from IP geolocation.

## Hardware required

The code indicates the following intended bill of materials:

| Qty | Part | Purpose / notes |
|---:|---|---|
| 1 | **ESP32-S3-DevKitC-1** | Main controller and Wi-Fi web server. This is the board selected in `platformio.ini`. |
| 2 | **Bipolar stepper motors** | Azimuth and elevation axes. The source comments mention a 28BYJ-48, but see the compatibility note below. |
| 2 | **A4988 stepper-driver modules** | One driver per motor. The firmware sets STEP, DIR, and ENABLE signals. |
| 1 | **BNO055 IMU module** | Heading/orientation feedback over I2C. |
| 1 | **AS5600 magnetic encoder module** | Elevation-angle feedback over I2C. Requires a correctly mounted diametrically magnetized magnet. |
| 1 | **Two-axis pan/tilt mount** | Mechanical azimuth/elevation structure, with suitable brackets, bearings, couplers, and gearing. |
| 1 | **External motor power supply** | Must match the selected motors and driver configuration; size it for both motors. Do not power the motors from the ESP32 board. |
| 1 | **USB data cable** | Firmware upload and serial monitoring. |
| As needed | Jumper wires, connectors, prototyping board, mounting hardware | Wiring and mechanical assembly. |

### Important hardware compatibility and safety notes

1. **28BYJ-48 compatibility:** a common stock 28BYJ-48 is a five-wire unipolar motor. An A4988 is intended for a four-wire bipolar stepper. Use a genuine four-wire bipolar motor, or deliberately convert a compatible 28BYJ-48 to bipolar wiring before connecting it. Do not connect a stock five-wire motor to an A4988 as-is.
2. **Motor power:** use a separate, correctly rated motor supply and connect its ground to ESP32 ground. Set each A4988 current limit for the actual motor. Fit the driver's recommended supply decoupling capacitor close to each driver and provide cooling as required.
3. **Logic levels:** ESP32-S3 GPIOs are 3.3 V logic and are not 5 V tolerant. Ensure the BNO055 and AS5600 I2C pull-ups do not exceed 3.3 V; use suitable level shifting if required by the modules.
4. **Current ENABLE assignments:** the firmware branch uses GPIO16 (azimuth ENABLE) and GPIO17 (elevation ENABLE), as approved for the ESP32-S3-DevKitC-1. Verify the exact module variant and board revision before wiring.
5. **I2C pins:** the firmware explicitly initializes SDA=GPIO8 and SCL=GPIO9. Connect only the BNO055 and AS5600 to this shared sensor bus. The LCD has been removed from the firmware and is not part of the required hardware.
6. Add mechanical hard stops, a physical emergency stop, and preferably limit switches before allowing movement. Limit-switch and emergency-stop handling is not implemented in this firmware. **Important:** when a fault occurs or the target is below the horizon, the firmware disables the A4988 drivers. Disabling a driver removes motor holding torque; counterbalance the elevation axis or add a suitable brake so the telescope cannot fall.

## Current firmware pin definitions

These are the definitions currently in `src/spacepointer_ver_2.cpp`; they are **not a validated wiring diagram**.

| Axis | STEP | DIR | ENABLE | Review |
|---|---:|---:|---:|---|
| Azimuth | GPIO12 | GPIO13 | GPIO16 | Firmware assignment on the approved ESP32-S3 configuration. |
| Elevation | GPIO14 | GPIO15 | GPIO17 | Firmware assignment on the approved ESP32-S3 configuration. |

The source comments configure A4988 MS1, MS2, and MS3 high for 1/16 microstepping. Confirm the actual driver-board pinout and motor/gearbox step count. The `MICROSTEPS_PER_REV` constant is currently not used to convert pointing angles into calibrated mechanical positions.

## Manual settings to configure in the source

Before operating the mount, review these values near the top of `src/spacepointer_ver_2.cpp`. They are intentionally kept as simple constants so the installation-specific settings are easy to find.

| Setting | Current default | What you must enter / verify |
|---|---:|---|
| `MAGNETIC_DECLINATION_DEG` | `0.0f` | **Enter the magnetic declination at the actual telescope installation location**, in degrees. East is positive; west is negative. Obtain a current value from a trusted magnetic-field model for the installation coordinates and date. The BNO055 measures magnetic heading, while the calculated target azimuth is relative to true north. Zero is only correct where magnetic and true north coincide. |
| `ENCODER_ZERO_RAW` | `2048` | **Calibrate this value.** Place the telescope tube physically level (0° elevation), read `encoder.rawAngle()` (0–4095) from the sensor, and enter that reading. 2048 is only a placeholder inherited from the old half-turn assumption; it is not a measured calibration. |
| `ENCODER_DIRECTION` | `1` | **Verify this value.** Move the telescope upward a small amount and observe whether the raw encoder reading increases. Use `1` if it increases, or `-1` if it decreases. Direction is applied in software; do not also call `encoder.setDirection()`. |
| `CONTROL_USER` | `admin` | Change the web dashboard username before deployment. |
| `CONTROL_PASSWORD` | `spacepointer-change-me` | **Change this password** to a unique, strong password. Basic Authentication is sent over plain HTTP in this firmware, so it does not protect credentials from someone able to monitor the local network. |
| `SETUP_AP_PASSWORD` | `SpacePointerSetup` | Change the WiFiManager setup access-point password. Use a password supported by the Wi-Fi stack (at least 8 characters). |
| `current` initial target | ISS / NORAD `25544` | Optional: change the default satellite name and NORAD catalog ID if you want another initial target. |
| `MICROSTEPS_PER_REV` | `32768UL` | Check against the actual motor full-step angle, gearbox ratio, and A4988 microstep setting. **This constant is currently not used for angular positioning**, so changing it alone does not calibrate the mount. |
| `STEP_PULSE_US` / `MOTOR_TASK_MS` / `MAX_STEPS_PER_TICK` | `50` µs / `25` ms / `8` steps | Pulse timing and a conservative per-tick correction cap. The cap limits large target changes; it is not an acceleration ramp. Change it only after unloaded bench testing. |
| `pidAz.kp` | `2.5` | Proportional gain for the azimuth axis, in **steps per degree**. The controller multiplies the azimuth pointing error (degrees) by this value to calculate a requested number of motor steps. A larger value commands more steps for the same error; a smaller value makes corrections gentler. This is an experimental starting value, not a mechanically calibrated value. |
| `pidEl.kp` | `3.0` | Proportional gain for the elevation axis, also in **steps per degree**. It has the same role as `pidAz.kp`, but can be tuned independently because the two axes may have different motors, gearing, and loads. This is an experimental starting value, not a mechanically calibrated value. |
| `pidAz.deadband` / `pidEl.deadband` | `0.05°` / `0.05°` | Minimum error (in degrees) that must be exceeded before that axis requests movement. Errors at or below the deadband are treated as zero, helping prevent motor chatter caused by small sensor fluctuations. A larger deadband reduces unnecessary corrections but allows more pointing error; a smaller one tracks more tightly but may react to noise. |

### Safe AS5600 calibration procedure

1. Keep motor power disconnected and place the telescope tube level using a reliable physical level.
2. Read the AS5600 raw angle over Serial (or temporarily print `encoder.rawAngle()` after a successful read).
3. Copy that exact 0–4095 reading into `ENCODER_ZERO_RAW`.
4. Move the tube upward by hand a small, safe amount and check the raw reading. Set `ENCODER_DIRECTION` to `1` or `-1` as described above.
5. Rebuild and verify the reported elevation manually at level and at a second known angle before connecting motor power.

**Important:** this software calibration establishes the encoder's angular zero and sign only. It does not establish mechanical travel limits, perform homing, or prevent the mount from hitting a hard stop. Do not infer safe travel limits from these settings.

### Location and network settings

- Wi-Fi credentials are provisioned through the `SpaceTracker_AP` WiFiManager setup portal; they are not hard-coded in this source file.
- The initial IP-geolocation fallback assigns altitude `50.0` metres. It is an estimate, not a site-specific surveyed altitude. Use the phone location update and verify its altitude if accurate observer height matters.
- The magnetic declination must correspond to the physical installation site, not the phone's temporary location, unless the mount itself is moved.
- The firmware currently uses an HTTP local dashboard. Browser geolocation may be unavailable in that insecure context; do not assume that the browser GPS button will work in every browser.


## Software and dependencies

- PlatformIO (PlatformIO Core or the VS Code PlatformIO extension)
- Espressif32 platform with the Arduino framework
- Libraries declared in `platformio.ini`:
  - WiFiManager
  - ESPAsyncWebServer
  - ArduinoJson
  - Adafruit BNO055, Adafruit Unified Sensor, and Adafruit BusIO
  - Sgp4
  - RobTillaart AS5600
- Astronomy Engine is included locally under `lib/astronomy/`.

### Build and upload

Open the repository folder in VS Code with PlatformIO, or use the PlatformIO CLI from the project root:

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

Run the native helper tests as well as the ESP32 build:
```sh
pio test -e native
pio run -e esp32-s3-devkitc-1
```

The GitHub Actions workflow runs both commands on pushes to the fix branch and on pull requests. A green CI result verifies compilation and the hardware-independent helper tests; it does not replace bench testing, sensor calibration, or safe motor commissioning.

## First boot and operation

1. Connect the ESP32-S3 to the computer and open the serial monitor at **115200 baud**.
2. Configure Wi-Fi using the WiFiManager setup access point named `SpaceTracker_AP`. Its current default password is `SpacePointerSetup`.
3. Once connected to the same Wi-Fi network, find the ESP32's DHCP address in the router's client list and open `http://<ESP32-IP>/` in a browser.
4. Select a listed satellite or planet, or enter a NORAD ID to request a satellite.
5. Use **Sync Phone GPS** to send the phone's permitted location to the device. Browser geolocation requires user permission and may be restricted by browsers when the page is served over plain HTTP.
6. The satellite TLE, IP geolocation, NTP time, Leaflet assets, and map tiles require internet access.

The web server listens on port 80. The dashboard root (`/`), `/set`, and `/updateLoc` require HTTP Basic Authentication. Opening the dashboard prompts the browser to authenticate before its JavaScript can call the control endpoints. The current source defaults are username `admin` and password `spacepointer-change-me`; change both control credentials and the setup AP password before deployment. Basic Authentication over plain HTTP does not encrypt credentials, so keep this device on a trusted local network and do not expose port 80 to the internet. The read-only `/data` endpoint remains unauthenticated.

## Implemented fixes and remaining commissioning work

The `fix/satellite-nextpass-minimum-elevation` branch includes these software changes:

- Corrected the AS5600 PlatformIO dependency and added the required I2C include.
- Changed driver ENABLE pins to GPIO16 and GPIO17; explicitly selected I2C GPIO8/GPIO9.
- Replaced the derivative/integral PID with a small proportional controller: this avoids derivative kick on target changes, keeps normal tracking corrections small, and caps unusually large corrections per control tick.
- Corrected the Moon and Mars body IDs (Moon=10, Mars=3) and the one-second planetary look-ahead using `Astronomy_AddDays`.
- Added a TLE fetch/validation step before satellite calculations, serialized access to SGP4, and restored SGP4 to the current epoch after the velocity look-ahead.
- Protected target, observer/telemetry, I2C, and SGP4 shared state with mutexes and used snapshots between tasks.
- Added bounds checks for observer coordinates, NORAD IDs, and supported planetary bodies; added HTTP response/JSON/TLE validation.
- Kept motor drivers disabled until time, location, orbital solution (for satellites), and both feedback sensors are ready; runtime motor GPIO writes are now owned exclusively by MotorTask, while other tasks can only request a stop. Sensor/mutex failures and below-horizon targets stop step pulses and disable the drivers. If NTP synchronizes after the startup timeout, the physics task can now resume automatically.
- Updated the AS5600 read call to the library's actual `rawAngle()` API and use `lastError()` to reject a failed I2C read.
- Added configurable AS5600 zero-angle and direction calibration, and converted encoder readings relative to the calibrated horizontal position.
- Added magnetic-declination compensation to convert BNO055 magnetic heading to true-north heading (the default declination remains a placeholder that must be configured for the installation site).
- Corrected satellite next-pass prediction to pass the 20-degree minimum elevation to the proper SGP4 argument and check prediction initialization.
- Removed the physical LCD and its display task. Satellite-pass and planetary rise/set calculations now run in the physics task and remain available through the web telemetry API.
- Fixed satellite and planetary telemetry population and added the missing planetary set-time field to `/data`.
- Added HTTP Basic Authentication to the target/location control endpoints, password-protected WiFiManager setup AP, native helper tests, and GitHub Actions build/test jobs.

### Still required before hardware operation

- Verify the physical ESP32-S3 module variant, GPIO16/17 availability, and all wiring against the actual board.
- Use four-wire bipolar steppers with A4988 drivers, set driver current limits for the exact motor model, and use a correctly rated separate motor supply. The common stock five-wire 28BYJ-48 is not directly A4988-compatible.
- Confirm the AS5600 magnet orientation, BNO055 mounting/alignment, motor directions, azimuth reference, elevation zero, gear ratios, microstep setting, and physical travel range. The firmware does not yet provide a full mount calibration wizard or closed-loop homing.
- Add physical limit switches and an emergency stop. They are not implemented in software yet. A4988 disable removes holding torque; balance the elevation axis or use a brake to prevent it dropping when the driver is disabled.
- Replace the example AP and web credentials in the source before deployment. HTTP Basic Authentication is not encrypted on the plain-HTTP connection.
- IP geolocation currently uses the IP-API HTTP endpoint. Treat this as an approximate startup location; use the phone GPS update for a more accurate observer position.
- The default UI page now uses HTTP Basic Authentication too, so the browser authenticates before control fetches are made.
- Confirm the SGP4 library's returned altitude/distance units against its documentation and a known satellite pass; the current telemetry code retains the library's documented/public fields and assumes kilometres.
- Complete a staged bench test: build, sensor-only checks, motor-driver tests with the mount unloaded, then slow tracking tests with hard stops and emergency stop available.

### Validation status

A successful CI build and native test run should be checked on the pull request before merging. No physical ESP32-S3, motor, encoder, or mount test has been performed by this code review.

## Repository layout

```text
.
├── platformio.ini
├── src/
│   └── spacepointer_ver_2.cpp
├── lib/
│   └── astronomy/
│       ├── astronomy.c
│       └── astronomy.h
├── include/
└── test/
    └── test_tracking_math/
        └── test_main.cpp
```

## References

- [ESP32-S3-DevKitC-1 hardware guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide.html)
- [ESP32-S3 GPIO guidance](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/gpio.html)
- [Arduino-ESP32 I2C API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html)
- [PlatformIO project configuration](https://docs.platformio.org/page/projectconf.html)
