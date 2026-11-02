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
