#include "../include/ControlBoardDriver.hh"

#include "../include/ControlBoardData.hh"
#include "../include/ControlBoardDataSingleton.hh"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <mutex>

namespace yarp
{
namespace dev
{
namespace gzyarp
{

// DeviceDriver
bool ControlBoardDriver::open(yarp::os::Searchable& config)
{
    m_controlBoardScopedName = config.find(YarpControlBoardScopedName).asString();

    m_controlBoardData
        = ::gzyarp::ControlBoardDataSingleton::getControlBoardHandler()->getControlBoardData(
            m_controlBoardScopedName);
}

bool ControlBoardDriver::close()
{
    return true;
}

bool ControlBoardDriver::getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    try
    {
        *mode
            = m_controlBoardData->joints.at(m_controlBoardData->getJointName(axis)).interactionMode;
    } catch (const std::exception& e)
    {
        yError() << "Error while getting interaction mode for axis " + std::to_string(axis) + ": \n"
                        + e.what();
        return false;
    }

    return true;
}

bool ControlBoardDriver::getInteractionModes(int n_joints,
                                             int* joints,
                                             yarp::dev::InteractionModeEnum* modes)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!joints)
    {
        yError() << "Error while getting interaction modes: joints array is null";
        return false;
    }
    if (!modes)
    {
        yError() << "Error while getting interaction modes: modes array is null";
        return false;
    }

    for (int i = 0; i < n_joints; i++)
    {
        if (!ControlBoardDriver::getInteractionMode(joints[i], &modes[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!modes)
    {
        yError() << "Error while getting interaction modes: modes array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getInteractionMode(i, &modes[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    try
    {
        m_controlBoardData->joints.at(m_controlBoardData->getJointName(axis)).interactionMode
            = mode;
    } catch (const std::exception& e)
    {
        yError() << "Error while setting interaction mode for axis " + std::to_string(axis) + ": \n"
                        + e.what();
        return false;
    }

    return true;
}

bool ControlBoardDriver::setInteractionModes(int n_joints,
                                             int* joints,
                                             yarp::dev::InteractionModeEnum* modes)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!joints)
    {
        yError() << "Error while setting interaction modes: joints array is null";
        return false;
    }
    if (!modes)
    {
        yError() << "Error while setting interaction modes: modes array is null";
        return false;
    }

    for (int i = 0; i < n_joints; i++)
    {
        if (!ControlBoardDriver::setInteractionMode(joints[i], modes[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!modes)
    {
        yError() << "Error while setting interaction modes: modes array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::setInteractionMode(i, modes[i]))
        {
            return false;
        }
    }

    return true;
}

} // namespace gzyarp
} // namespace dev
} // namespace yarp
