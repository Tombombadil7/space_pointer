#pragma once

#include <math.h>

namespace TrackingMath {

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
    if (!isfinite(errorDegrees)) return 0;
    if (fabsf(errorDegrees) <= deadband) return 0;

    const long requestedSteps = lroundf(gain * errorDegrees);
    if (requestedSteps > maxSteps) return maxSteps;
    if (requestedSteps < -maxSteps) return -maxSteps;
    return (int)requestedSteps;
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

inline bool supportedPlanetBody(int body) {
    return body == 3 || body == 4 || body == 5 || body == 10;
}

} // namespace TrackingMath
