#pragma once

#include <map>
#include <mutex>
#include <string>

#include <yarp/dev/IInteractionMode.h>

struct JointProperties
{
    std::string jointName;
    yarp::dev::InteractionModeEnum interactionMode;
};

class ControlBoardData
{
public:
    std::mutex mutex;
    std::string modelScopedName;
    std::map<std::string, JointProperties> joints;

    std::string getJointName(const int& _jointIndex) const;
};
