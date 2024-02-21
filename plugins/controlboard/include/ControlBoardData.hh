#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/conf/numeric.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/PidEnums.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Vocab.h>

#include <gz/math/PID.hh>

namespace gzyarp
{

struct PidControlTypeEnumHashFunction
{
    size_t operator()(const yarp::dev::PidControlTypeEnum& key) const
    {
        std::size_t hash = std::hash<int>()(static_cast<int>(key));
        return hash;
    }
};

struct JointProperties
{
    std::string name;
    yarp::dev::InteractionModeEnum interactionMode;
    yarp::conf::vocab32_t controlMode;
    // TODO(xela95): Update unit of measurements once we support prismatic joints
    double refTorque; // Desired reference torques for torque control mode [Nm]
    double torque; // Measured torques [Nm]
    double maxTorqueAbs; // Maximum torque absolute value [Nm]
    double zeroPosition; // The zero position is the position of the GAZEBO joint that will be read
                         // as the starting one i.e.
                         // getEncoder(j)=m_zeroPosition+gazebo.getEncoder(j);
    double refPosition;
    double position; // Joint position [deg]
    double positionLimitMin;
    double positionLimitMax;
    double velocity; // Joint velocity [deg/s]
    double velocityLimitMin;
    double velocityLimitMax;
    std::unordered_map<yarp::dev::PidControlTypeEnum, gz::math::PID, PidControlTypeEnumHashFunction>
        pidControllers;
};

class ControlBoardData
{
public:
    std::mutex mutex;
    std::string modelScopedName;
    std::vector<JointProperties> joints;
    yarp::os::Stamp simTime;
};

} // namespace gzyarp
