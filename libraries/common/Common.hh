#pragma once

namespace gzyarp
{
constexpr double pi = 3.1415926535897932384626433;

inline double convertDegreesToRadians(double degrees)
{
    return degrees / 180.0 * pi;
}

inline double convertRadiansToDegrees(double radians)
{
    return radians * 180.0 / pi;
}

// Convert a degree gain expressed in [Nm/°] to a radian gain expressed in [Nm/rad]
inline double convertDegreeGainToRadianGains(double degreeGain)
{
    return degreeGain * 180.0 / pi;
}

// Convert a radian gain expressed in [Nm/rad] to a degree gain expressed in [Nm/°]
inline double convertRadianGainToDegreeGains(double radianGain)
{
    return radianGain / 180.0 * pi;
}

} // namespace gzyarp
