# Universal Space Pointer

An experimental ESP32-S3 project for pointing a two-axis motorized mount at selected satellites and solar-system objects. The firmware combines orbital/astronomical calculations with an IMU, a magnetic rotary encoder, stepper-motor drivers, an LCD, and a browser-based control panel.

> **Project status:** prototype / work in progress. This README documents the current repository. A static code review found build, pin-mapping, and runtime issues that should be resolved before connecting motor power. A successful PlatformIO build or full hardware test has not been verified.

## Intended features

- Track satellites by NORAD catalog ID using downloaded TLE data and the SGP4 library.
- Calculate pointing directions for the Moon, Mars, Jupiter, and Saturn using the bundled Astronomy Engine library.
- Drive azimuth and elevation stepper motors with A4988 drivers.
- Use a BNO055 IMU for heading feedback and an AS5600 magnetic encoder for elevation feedback.
- Display pointing and telemetry information on a 16x2 I2C LCD.
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
| 1 | **16x2 character LCD with I2C backpack** | Local status display. Firmware probes address `0x27`, then `0x3F`. |
| 1 | **Two-axis pan/tilt mount** | Mechanical azimuth/elevation structure, with suitable brackets, bearings, couplers, and gearing. |
| 1 | **External motor power supply** | Must match the selected motors and driver configuration; size it for both motors. Do not power the motors from the ESP32 board. |
| 1 | **USB data cable** | Firmware upload and serial monitoring. |
| As needed | Jumper wires, connectors, prototyping board, mounting hardware | Wiring and mechanical assembly. |

### Important hardware compatibility and safety notes

1. **28BYJ-48 compatibility:** a common stock 28BYJ-48 is a five-wire unipolar motor. An A4988 is intended for a four-wire bipolar stepper. Use a genuine four-wire bipolar motor, or deliberately convert a compatible 28BYJ-48 to bipolar wiring before connecting it. Do not connect a stock five-wire motor to an A4988 as-is.
2. **Motor power:** use a separate, correctly rated motor supply and connect its ground to ESP32 ground. Set each A4988 current limit for the actual motor. Fit the driver's recommended supply decoupling capacitor close to each driver and provide cooling as required.
3. **Logic levels:** ESP32-S3 GPIOs are 3.3 V logic and are not 5 V tolerant. Check the LCD backpack's I2C pull-up voltage; use 3.3 V pull-ups or a suitable bidirectional I2C level shifter if the backpack pulls SDA/SCL up to 5 V.
4. **Do not wire the current ENABLE pin assignments yet.** The code currently assigns azimuth ENABLE to GPIO25 and elevation ENABLE to GPIO26. GPIO25 is not exposed as a usable GPIO on the ESP32-S3-DevKitC-1, and GPIO26–GPIO32 are normally reserved for SPI flash/PSRAM on ESP32-S3 modules. Reassign both ENABLE signals to verified, exposed GPIOs before powering the drivers.
5. **I2C pins:** the source comment says SDA=21 and SCL=22, which is the familiar generic-ESP32 mapping, not the ESP32-S3-DevKitC-1 Arduino default. The S3 board mapping uses SDA=8 and SCL=9. Wire the BNO055, AS5600, and LCD to the same I2C bus, and preferably make the pin choice explicit in code with `Wire.begin(8, 9)` (or another verified mapping).
6. Add mechanical hard stops, a physical emergency stop, and preferably limit switches before allowing unattended movement. Limit-switch and emergency-stop handling is not implemented in the current firmware.

## Current firmware pin definitions

These are the definitions currently in `src/spacepointer_ver_2.cpp`; they are **not a validated wiring diagram**.

| Axis | STEP | DIR | ENABLE | Review |
|---|---:|---:|---:|---|
| Azimuth | GPIO12 | GPIO13 | GPIO25 | STEP/DIR are exposed; GPIO25 is not a usable S3 DevKitC-1 pin. |
| Elevation | GPIO14 | GPIO15 | GPIO26 | STEP/DIR are exposed; GPIO26 is normally reserved for flash/PSRAM. |

The source comments configure A4988 MS1, MS2, and MS3 high for 1/16 microstepping. Confirm the actual driver-board pinout and motor/gearbox step count. The `MICROSTEPS_PER_REV` constant is currently not used to convert pointing angles into calibrated mechanical positions.

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
  - LiquidCrystal_I2C
- Astronomy Engine is included locally under `lib/astronomy/`.

### Build and upload

Open the repository folder in VS Code with PlatformIO, or use the PlatformIO CLI from the project root:

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

**The current configuration/source needs the fixes listed in “Known issues” before expecting a successful build.** In particular, correct the AS5600 dependency entry and add the missing LiquidCrystal_I2C header include. These build commands are instructions, not a claim that the current commit has already compiled successfully.

## First boot and operation

1. Connect the ESP32-S3 to the computer and open the serial monitor at **115200 baud**.
2. Configure Wi-Fi using the WiFiManager setup access point named `SpaceTracker_AP` when the device has no saved network credentials.
3. Once connected to the same Wi-Fi network, find the ESP32's DHCP address in the router's client list and open `http://<ESP32-IP>/` in a browser.
4. Select a listed satellite or planet, or enter a NORAD ID to request a satellite.
5. Use **Sync Phone GPS** to send the phone's permitted location to the device. Browser geolocation requires user permission and may be restricted by browsers when the page is served over plain HTTP.
6. The satellite TLE, IP geolocation, NTP time, Leaflet assets, and map tiles require internet access.

The web server listens on port 80. The HTML page calls the `/data`, `/set`, and `/updateLoc` endpoints.

## Known issues found in the code review

These are findings from static inspection of the current source, not a compiler or bench-test report.

### Build blockers

- In `platformio.ini`, the AS5600 dependency line begins with the stray word `Installing`. It should be a PlatformIO library specification such as `RobTillaart/AS5600 @ ^0.6.2`.
- `src/spacepointer_ver_2.cpp` declares and uses `LiquidCrystal_I2C` but does not include `<LiquidCrystal_I2C.h>`. Add the header (and include `<Wire.h>` explicitly for clarity).

### Hardware / motor-control blockers

- The ENABLE pins (GPIO25 and GPIO26) are unsuitable for the configured ESP32-S3-DevKitC-1. Reassign them before wiring.
- The firmware starts the motor task even when the BNO055 or AS5600 is not detected. It also enables the A4988 drivers during setup. A failed sensor read or a failed mutex acquisition can leave the local sensor event uninitialized and then feed an undefined reading into the PID controller. Add sensor-validity checks and a fail-safe that disables the drivers.
- Motor direction, mechanical zero, gear ratio, travel limits, and encoder alignment are not calibrated by the code. The motor/gearbox assumptions need to be verified on the actual mount.
- **PID initialization is misordered.** The `PID` struct fields are `integral, lastErr, kp, ki, kd, deadband`, but the two four-value aggregate initializers provide values as if the first fields were `kp, ki, kd, deadband`. As written, `kp` becomes 0.4 / 0.5, `ki` becomes 1.5 / 1.0, `kd` stays zero, and `deadband` stays zero; the integral and previous-error state start nonzero. Initialize all six fields explicitly (with integral and lastErr set to zero) before tuning.

### Target-selection and calculation bugs

- The planet buttons pass incorrect Astronomy Engine body IDs: in the bundled header, `BODY_MERCURY=0`, `BODY_VENUS=1`, `BODY_MARS=3`, `BODY_JUPITER=4`, `BODY_SATURN=5`, and `BODY_MOON=10`. The current UI sends 0 for Moon and 1 for Mars, so those buttons select Mercury and Venus respectively.
- The planet speed calculation builds the second time with `tm_sec + 1.0/86400.0`. `Astronomy_MakeTime` expects seconds in that argument, so this advances by about 11.6 microseconds, not one second. Use `tm_sec + 1.0` or `Astronomy_AddDays(aTime, 1.0/86400.0)`.
- The startup path does not fetch the initial ISS TLE before starting `PhysicsTask`. The SGP4 object can therefore be used before it has been initialized; fetch and validate the initial TLE before starting the tracking task.
- For satellites, the LCD reads `sys.satAltKm` and `sys.satDist`, but the satellite branch of `PhysicsTask` does not populate those fields. The LCD can consequently display zero/stale values even when the web page has satellite data.
- For planets, `/data` sends altitude and distance from the SGP4 object rather than the planet values calculated into `sys`, so the browser can show stale satellite telemetry. The response also omits `nextPassLOS`, although the browser expects it for the planet “Sets at” display.

### Concurrency, validation, and security

- The asynchronous web handlers, `fetchTLE`, `PhysicsTask`, and `DisplayTask` share `current`, `satSgp4`, and several `sys` fields without a common lock. Target changes or TLE downloads can race with calculations and display updates.
- Validate NORAD IDs, target types, and location ranges before accepting them. Check HTTP status and JSON deserialization results, and verify that TLE line pointers are non-null before copying them.
- The local web endpoints that change target/location have no authentication. The setup AP is created without an AP password in the current `autoConnect` call. Keep the device on a trusted network; do not expose it directly to the internet.
- The `test/` directory contains only PlatformIO's placeholder README; there are no project tests or CI workflows in the repository.

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
```

## References

- [ESP32-S3-DevKitC-1 hardware guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide.html)
- [ESP32-S3 GPIO guidance](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/gpio.html)
- [Arduino-ESP32 I2C API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html)
- [PlatformIO project configuration](https://docs.platformio.org/page/projectconf.html)
