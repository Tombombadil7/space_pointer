# ESP32-S3 hardware bench test plan

These checks require the real ESP32-S3, BNO055, AS5600, A4988 drivers, motors,
and a phone on the device's Wi-Fi network. They are deliberately not reported
as passed by the desktop CI.

## Safety before applying motor power

- Secure the mount and test with no telescope/payload attached.
- Keep the mechanism clear of hands, cables, and hard stops.
- Use a current-limited motor supply and verify driver wiring/current settings
  against the actual motor and A4988 carrier before powering it.
- Start with motor power disconnected. Confirm the ESP32 logic and sensors first.
- Have a physical way to remove motor power immediately.
- Power down before disconnecting/reconnecting I2C sensors or changing wiring.

## Test sequence

1. **Boot with motor supply disconnected**
   - Confirm the ESP32 starts and the web page loads.
   - Confirm the serial log reports the BNO055 and AS5600 as detected.
   - Measure AZ_EN and EL_EN: both must remain HIGH (disabled) during startup,
     Wi-Fi setup, missing time/location, and before a valid tracking solution.

2. **I2C and sensor availability**
   - Confirm BNO055 (0x28) and AS5600 (0x36) are both visible on SDA GPIO8 /
     SCL GPIO9.
   - With motor power still disconnected, power down and disconnect one sensor
     at a time, then reboot. The corresponding readiness warning must appear
     and the drivers must remain disabled.
   - Repeat with a sensor that is detected but not returning valid readings.

3. **BNO055 calibration gate**
   - Keep motor power disconnected.
   - Observe system and magnetometer calibration levels below 3. The drivers
     must remain disabled.
   - Only after both report 3 may the sensor sample pass the calibration gate.
   - Rotate slowly through north (359° to 0° to 1°); heading must remain finite
     and continuous.

4. **AS5600 elevation boundaries**
   - With the mount supported and motor power disconnected, move the sensor
     through the calibrated level position.
   - Verify the configured zero and direction match the physical movement.
   - Check the expected 0° and 90° positions.
   - A reading below 0°, above 90°, invalid raw value, or bad calibration must
     keep both drivers disabled. Do not force the mechanism against a hard stop.

5. **Driver pulse verification**
   - First verify signals with motor power disconnected using a logic analyzer
     or oscilloscope.
   - DIR must be set before each STEP pulse.
   - STEP high and low intervals are each configured for 50 microseconds.
   - No more than 8 steps per axis may be requested in one 25 ms motor tick.
   - Confirm ENABLE is active-low and both drivers are disabled whenever any
     readiness or sensor prerequisite is missing.
   - Then, with the mechanism clear and current limited, test one axis at a
     time using a very small target correction. Confirm direction before
     allowing the second axis to move.

6. **Target/location changes during tracking**
   - While a small correction is being made, change the selected target.
   - Repeat while syncing a new phone location.
   - Old-generation commands must not continue after the change; the drivers
     should stop until a calculation for the new target/location is ready.

7. **Satellite/TLE failure cases**
   - Test a valid NORAD ID, an unsupported/out-of-range ID, an unavailable TLE
     server, a timeout, and malformed/incomplete TLE response if a controlled
     test endpoint is available.
   - On any active-satellite TLE failure, the drivers must remain disabled.
   - Switch from a satellite to a planet while a TLE refresh is in flight; the
     stale satellite result must not replace the planet's state.

8. **Phone and web API**
   - From the phone, select each supported planet and a satellite.
   - Confirm state changes are POST requests and include
     `X-SpacePointer-Request: 1`.
   - Try missing, malformed, non-finite, and out-of-range coordinates/IDs using
     a controlled test client; expect HTTP 400/403 as appropriate.
   - Confirm a successful target/location update returns HTTP 202 and that
     `/data` returns a consistent target/telemetry pair.
   - Deny GPS permission, disable location services, and test a successful GPS
     sync. Record the browser behavior separately from the server response.

9. **Network interruption and recovery**
   - Temporarily disconnect the phone from Wi-Fi while the page is open.
   - The telemetry error should remain confined to the telemetry panel.
   - Reconnect and confirm telemetry and the satellite map recover.
   - Repeat with a temporary loss of internet access to distinguish local
     device communication from external map tiles/TLE services.

10. **Soak test**
    - Run a safe, unloaded tracking session for at least 30–60 minutes.
    - Watch for repeated I2C errors, watchdog resets, heap degradation, task
      starvation, unexpected driver enable/disable cycles, and stale telemetry.
    - Record serial output, supply voltage/current, and any missed or delayed
      movements.

## Record for each run

- Firmware commit SHA and ESP32 board revision.
- Motor supply voltage/current limit and A4988 current setting.
- Encoder zero/direction calibration.
- BNO055 calibration status.
- Test case, expected result, observed result, and serial log excerpt.
