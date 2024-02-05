#pragma once

#include <map>
#include <mutex>
#include <string>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/os/Vocab.h>

struct JointProperties
{
    std::string jointName;
    yarp::dev::InteractionModeEnum interactionMode;
    yarp::conf::vocab32_t controlMode;
};

class ControlBoardData
{
public:
    std::mutex mutex;
    std::string modelScopedName;
    std::map<std::string, JointProperties> joints;

    std::string getJointName(const int& jointIndex) const;
    int getJointIndex(const std::string& jointName) const;
};
