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

} // namespace gzyarp
