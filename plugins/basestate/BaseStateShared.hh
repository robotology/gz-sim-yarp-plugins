#pragma once

#include <array>
#include <mutex>
#include <string>
#include <yarp/os/Stamp.h>

namespace gzyarp
{

struct BaseStateData
{
    std::mutex mutex;
    std::string baseLinkScopedName;
    bool dataAvailable;
    // worldBasePose.Pos()
    std::array<double, 3> position; // [x, y, z] in meters
    // worldBasePose.Rot()
    std::array<double, 3> orientation; // [roll, pitch, yaw] in radians
    // worldBaseLinVel
    std::array<double, 3> linVel; // [vx, vy, vz] in m/s
    // worldBaseAngVel
    std::array<double, 3> angVel; // [wx, wy, wz] in rad/s
    // worldBaseLinAcc
    std::array<double, 3> linAcc; // [ax, ay, az] in m/s^2
    // worldBaseAngAcc
    std::array<double, 3> angAcc; // [alphax, alphay, alphaz] in rad/s^2
    yarp::os::Stamp simTimestamp;
};

class IBaseStateData
{
public:
    virtual void setBaseStateData(BaseStateData*) = 0;

    virtual ~IBaseStateData() {};
};

} // namespace gzyarp
