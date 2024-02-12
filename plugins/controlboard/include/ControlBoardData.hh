#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <yarp/conf/numeric.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Vocab.h>

struct JointProperties
{
    std::string name;
    yarp::dev::InteractionModeEnum interactionMode;
    yarp::conf::vocab32_t controlMode;
    double refTorque; // Desired reference torques for torque control mode [Nm]
    double torque; // Measured torques [Nm]
    double maxTorqueAbs; // Maximum torque absolute value [Nm]
    double zeroPosition; // The zero position is the position of the GAZEBO joint that will be read
                         // as the starting one i.e.
                         // getEncoder(j)=m_zeroPosition+gazebo.getEncoder(j);
    double position; // Joint position [deg]
    double positionLimitMin;
    double positionLimitMax;
    double velocity; // Joint velocity [deg/s]
    double velocityLimitMin;
    double velocityLimitMax;
};

class ControlBoardData
{
public:
    std::mutex mutex;
    std::string modelScopedName;
    std::vector<JointProperties> joints;
    yarp::os::Stamp simTime;
};
