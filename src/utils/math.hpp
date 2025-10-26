#pragma once
#include "types.hpp"

inline double
convert_to_positive_radians(double angle)
{

    if (angle < 0) {
        return angle + 2 * M_PI;
    }
    return angle;
}

inline double
min_angle_difference(double angle1, double angle2)
{
    // https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point
    // Angles must be subtracted differently than other values.
    return atan2(sin(angle2 - angle1), cos(angle2 - angle1));
}

inline bool
above_epsilon(f64 epsilon, f64 val)
{
    return std::abs(val) > epsilon;
}
