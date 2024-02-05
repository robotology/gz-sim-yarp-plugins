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

// IInteractionMode

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

// IControlMode

bool ControlBoardDriver::getControlMode(int j, int* mode)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!mode)
    {
        yError() << "Error while getting control mode: mode array is null";
        return false;
    }

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting control mode: joint index out of range";
        return false;
    }

    *mode = m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).controlMode;

    return true;
}

bool ControlBoardDriver::getControlModes(int* modes)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!modes)
    {
        yError() << "Error while getting control modes: modes array is null";
        return false;
    }

    // for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    // {
    //     if (!ControlBoardDriver::getControlMode(i, &modes[i]))
    //     {
    //         return false;
    //     }
    // }

    for (auto& [key, value] : m_controlBoardData->joints)
    {
        if (!ControlBoardDriver::getControlMode(m_controlBoardData->getJointIndex(key),
                                                &modes[m_controlBoardData->getJointIndex(key)]))
        {
            yError() << "Error while getting control mode for joint " + key;
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getControlModes(const int n_joint, const int* joints, int* modes)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!joints)
    {
        yError() << "Error while getting control modes: joints array is null";
        return false;
    }
    if (!modes)
    {
        yError() << "Error while getting control modes: modes array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getControlMode(joints[i], &modes[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setControlMode(const int j, const int mode)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting control mode: joint index out of range";
        return false;
    }

    // Only accept supported control modes
    // The only not supported control mode is
    // (for now) VOCAB_CM_MIXED
    if (!(mode == VOCAB_CM_POSITION || mode == VOCAB_CM_POSITION_DIRECT || mode == VOCAB_CM_VELOCITY
          || mode == VOCAB_CM_TORQUE || mode == VOCAB_CM_MIXED || mode == VOCAB_CM_PWM
          || mode == VOCAB_CM_CURRENT || mode == VOCAB_CM_IDLE || mode == VOCAB_CM_FORCE_IDLE))
    {
        yWarning() << "request control mode " << yarp::os::Vocab32::decode(mode)
                   << " that is not supported by "
                   << " gz-sim-yarp-controlboard-system plugin.";
        return false;
    }

    m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).controlMode = mode;

    return true;
}

bool ControlBoardDriver::setControlModes(const int n_joint, const int* joints, int* modes)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!joints)
    {
        yError() << "Error while setting control modes: joints array is null";
        return false;
    }
    if (!modes)
    {
        yError() << "Error while setting control modes: modes array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setControlMode(joints[i], modes[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setControlModes(int* modes)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!modes)
    {
        yError() << "Error while setting control modes: modes array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::setControlMode(i, modes[i]))
        {
            return false;
        }
    }

    return true;
}

// IAxisInfo

bool ControlBoardDriver::getAxisName(int axis, std::string& name)
{
    // TODO integrate with IJointCoupled interface

    name = m_controlBoardData->getJointName(axis);

    return true;
}

bool ControlBoardDriver::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    // TODO integrate with IJointCoupled interface

    return yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
}

// ITorqueControl

// bool ControlBoardDriver::getAxes(int* ax)
// {
//     // TODO integrate with IJointCoupled interface
//     *ax = m_controlBoardData->joints.size();

//     return true;
// }

// bool ControlBoardDriver::getRefTorques(double* t)
// {

//     return true;
// }

} // namespace gzyarp
} // namespace dev
} // namespace yarp
