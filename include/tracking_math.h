#pragma once

#include <math.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

namespace TrackingMath {


/*
 * Strict numeric parsers shared by the HTTP handlers and native tests.
 * Reject partial strings, range errors, and non-finite floating-point values.
 */
inline bool parseFiniteDouble(const char *text, double &value) {
    if (text == NULL || *text == '\0') return false;
    errno = 0;
    char *end = NULL;
    const double parsed = strtod(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

inline bool parseLongInteger(const char *text, long &value) {
    if (text == NULL || *text == '\0') return false;
    errno = 0;
    char *end = NULL;
    const long parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || errno == ERANGE) return false;
    value = parsed;
    return true;
}

inline float wrapAngle(float degrees) {
    if (!isfinite(degrees)) return 0.0f;
    if (degrees > 180.0f || degrees < -180.0f) {
        degrees = fmodf(degrees, 360.0f);
        if (degrees > 180.0f) degrees -= 360.0f;
        if (degrees < -180.0f) degrees += 360.0f;
    }
    return degrees;
}

/*
 * Convert a pointing error into a bounded proportional motor-step request.
 *
 * This helper deliberately contains no GPIO or FreeRTOS code, which makes
 * the controller's rounding, deadband, and safety cap testable on a PC.
 */
inline int proportionalSteps(float gain, float deadband,
                             float errorDegrees, int maxSteps) {
    if (!isfinite(gain) || !isfinite(deadband) || deadband < 0.0f ||
        !isfinite(errorDegrees) || maxSteps <= 0) {
        return 0;
    }
    if (fabsf(errorDegrees) <= deadband) return 0;

    // Preserve the controller's original float multiplication/rounding,
    // but clamp before lroundf so extreme inputs cannot overflow conversion.
    const float requestedSteps = gain * errorDegrees;
    if (!isfinite(requestedSteps)) {
        return requestedSteps > 0.0f ? maxSteps :
               requestedSteps < 0.0f ? -maxSteps : 0;
    }
    if ((double)requestedSteps >= (double)maxSteps + 0.5) return maxSteps;
    if ((double)requestedSteps <= -(double)maxSteps - 0.5) return -maxSteps;

    const long roundedSteps = lroundf(requestedSteps);
    if (roundedSteps > maxSteps) return maxSteps;
    if (roundedSteps < -maxSteps) return -maxSteps;
    return (int)roundedSteps;
}

/*
 * Validate that a pair of astronomical events is in chronological order.
 * The firmware uses this after searching for a rise and then the following set.
 */
inline bool chronologicalEventPair(double firstEventTime, double secondEventTime) {
    return isfinite(firstEventTime) && isfinite(secondEventTime) &&
           secondEventTime > firstEventTime;
}

/*
 * Elevation is a bounded mechanical axis, not a circular angle.
 * Keep its validation and error calculation separate from azimuth wrapping.
 */
inline bool validElevation(float degrees) {
    return isfinite(degrees) && degrees >= 0.0f && degrees <= 90.0f;
}

inline float elevationError(float targetDegrees, float measuredDegrees) {
    return targetDegrees - measuredDegrees;
}


/*
 * Convert the AS5600's 12-bit circular reading into the telescope's bounded
 * elevation axis. Direction and zero are explicit calibration inputs.
 */
inline bool encoderElevation(uint16_t rawAngle, int zeroRaw, int direction,
                             float &elevationDegrees) {
    if (rawAngle > 4095 || zeroRaw < 0 || zeroRaw > 4095 ||
        (direction != 1 && direction != -1)) {
        return false;
    }

    int32_t delta = (int32_t)rawAngle - zeroRaw;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;

    elevationDegrees = direction * delta * (360.0f / 4096.0f);
    return validElevation(elevationDegrees);
}


/*
 * Centralize the exact conditions that must hold before MotorTask may
 * consider enabling the drivers. A satellite additionally needs valid TLE data.
 */
inline bool motorControlReady(bool targetRead, bool timeReady,
                              bool observerReady, bool calculationReady,
                              bool tleReadyForTarget, bool sensorsReady,
                              float targetAzimuth, float targetElevation) {
    return targetRead && timeReady && observerReady && calculationReady &&
           tleReadyForTarget && sensorsReady &&
           isfinite(targetAzimuth) && validElevation(targetElevation);
}

/*
 * Validate a complete sensor sample before using it to command the motors.
 * Calibration must be complete, heading must be a finite compass angle, and
 * the AS5600 must have returned a valid 12-bit reading.
 */
inline bool validMotorSensorSample(bool headingRead, bool encoderHealthy,
                                   int systemCalibration, int magneticCalibration,
                                   float magneticHeading, uint16_t encoderRaw) {
    return headingRead && encoderHealthy &&
           systemCalibration == 3 && magneticCalibration == 3 &&
           isfinite(magneticHeading) && magneticHeading >= 0.0f &&
           magneticHeading < 360.0f && encoderRaw <= 4095;
}

/* Normalize a magnetic heading into true-north degrees. */
inline float trueHeading(float magneticHeading, float declinationDegrees) {
    if (!isfinite(magneticHeading) || !isfinite(declinationDegrees)) return NAN;
    float heading = fmodf(magneticHeading + declinationDegrees, 360.0f);
    if (heading < 0.0f) heading += 360.0f;
    return heading;
}

/* Azimuth is circular, unlike the mechanically bounded elevation axis. */
inline float azimuthError(float targetDegrees, float measuredDegrees) {
    return wrapAngle(targetDegrees - measuredDegrees);
}

inline bool validObserver(double latitude, double longitude, double altitudeMeters) {
    return isfinite(latitude) && isfinite(longitude) && isfinite(altitudeMeters) &&
           latitude >= -90.0 && latitude <= 90.0 &&
           longitude >= -180.0 && longitude <= 180.0 &&
           altitudeMeters >= -500.0 && altitudeMeters <= 10000.0;
}

inline bool validNoradId(long noradId) {
    return noradId >= 1 && noradId <= 999999;
}

/* Validate the two fixed-width TLE lines against their embedded checksums
 * and the requested five-digit NORAD catalog number.
 */
inline bool validTleChecksum(const char *line) {
    if (line == NULL || strlen(line) < 69 ||
        line[68] < '0' || line[68] > '9') return false;

    int sum = 0;
    for (int i = 0; i < 68; ++i) {
        if (line[i] >= '0' && line[i] <= '9') sum += line[i] - '0';
        else if (line[i] == '-') ++sum;
    }
    return sum % 10 == line[68] - '0';
}

inline bool validTlePair(const char *line1, const char *line2,
                         long expectedNoradId) {
    if (!validNoradId(expectedNoradId) || line1 == NULL || line2 == NULL ||
        line1[0] != '1' || line2[0] != '2' ||
        !validTleChecksum(line1) || !validTleChecksum(line2)) {
        return false;
    }

    long line1Id = 0;
    long line2Id = 0;
    for (int i = 2; i <= 6; ++i) {
        if (line1[i] < '0' || line1[i] > '9' ||
            line2[i] < '0' || line2[i] > '9') return false;
        line1Id = line1Id * 10 + line1[i] - '0';
        line2Id = line2Id * 10 + line2[i] - '0';
    }
    return line1Id == line2Id && line1Id == expectedNoradId;
}

inline bool supportedPlanetBody(int body) {
    return body == 3 || body == 4 || body == 5 || body == 10;
}

/*
 * A calculation is usable only for the exact target and observer snapshots
 * from which it was produced. Zero is reserved to mean "not ready".
 *
 * This small value type has no RTOS dependency, so its state transitions can
 * be tested on a desktop build. In firmware, the caller must protect every
 * read/write with stateMutex.
 */
struct ConfigurationReadiness {
    uint32_t generation = 1;
    bool locationReady = false;
    bool tleReady = false;
    bool calculationReady = false;
    bool passReady = false;

    void clearResults() {
        tleReady = false;
        calculationReady = false;
        passReady = false;
    }

    void invalidate() {
        ++generation;
        if (generation == 0) ++generation;
        clearResults();
    }

    bool isCurrent(uint32_t candidate) const {
        return candidate != 0 && candidate == generation;
    }

    bool publishTle(uint32_t candidate) {
        if (!isCurrent(candidate)) return false;
        tleReady = true;
        return true;
    }

    bool publishCalculation(uint32_t candidate) {
        if (!isCurrent(candidate)) return false;
        calculationReady = true;
        return true;
    }

    bool publishPass(uint32_t candidate) {
        if (!isCurrent(candidate)) return false;
        passReady = true;
        return true;
    }
};

} // namespace TrackingMath
