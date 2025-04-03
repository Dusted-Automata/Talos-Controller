#pragma once
#include "types.hpp"

inline Ecef_Coord latLngToECEF(double lat, double lng, double alt = 0)
{
    // WGS84 ellipsoid constants
    double a = 6378137.0;         // semi-major axis in meters
    double e = 0.081819190842622; // first eccentricity

    // Convert latitude and longitude to radians
    double latRad = lat * (M_PI / 180.0);
    double lngRad = lng * (M_PI / 180.0);

    // Calculate N, the radius of curvature in the prime vertical
    double N = a / std::sqrt(1 - std::pow(e * std::sin(latRad), 2));

    // Calculate ECEF coordinates
    double x = (N + alt) * std::cos(latRad) * std::cos(lngRad);
    double y = (N + alt) * std::cos(latRad) * std::sin(lngRad);
    double z = (N * (1 - std::pow(e, 2)) + alt) * std::sin(latRad);

    return {x, y, z};
}
