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

    return true;
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

    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;

    return true;
}

// ITorqueControl

bool ControlBoardDriver::getAxes(int* ax)
{
    // TODO integrate with IJointCoupled interface

    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);
    *ax = m_controlBoardData->joints.size();

    return true;
}

bool ControlBoardDriver::getRefTorque(int j, double* t)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!t)
    {
        yError() << "Error while getting reference torque: t is null";
        return false;
    }
    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting reference torque: joint index out of range";
        return false;
    }

    *t = m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).refTorque;

    return true;
}

bool ControlBoardDriver::getRefTorques(double* t)
{
    if (!t)
    {
        yError() << "Error while getting reference torques: t array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getRefTorque(i, &t[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setRefTorque(int j, double t)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting reference torque: joint index out of range";
        return false;
    }

    m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).refTorque = t;

    return true;
}

bool ControlBoardDriver::setRefTorques(const double* t)
{
    if (!t)
    {
        yError() << "Error while setting reference torques: t array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::setRefTorque(i, t[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setRefTorques(const int n_joint, const int* joints, const double* t)
{
    if (!joints)
    {
        yError() << "Error while setting reference torques: joints array is null";
        return false;
    }
    if (!t)
    {
        yError() << "Error while setting reference torques: t array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setRefTorque(joints[i], t[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params)
{
    // TODO

    return false;
}

bool ControlBoardDriver::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    // TODO

    return false;
}

bool ControlBoardDriver::getTorque(int j, double* t)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!t)
    {
        yError() << "Error while getting torque: t is null";
        return false;
    }
    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting torque: joint index out of range";
        return false;
    }
    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting torque: joint index out of range";
        return false;
    }

    *t = m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).torque;

    return true;
}

bool ControlBoardDriver::getTorques(double* t)
{
    if (!t)
    {
        yError() << "Error while getting torques: t array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getTorque(i, &t[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getTorqueRange(int j, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!min)
    {
        yError() << "Error while getting torque range: min is null";
        return false;
    }
    if (!max)
    {
        yError() << "Error while getting torque range: max is null";
        return false;
    }
    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting torque range: joint index out of range";
        return false;
    }

    *min = -m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).maxTorqueAbs;
    *max = m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).maxTorqueAbs;

    return true;
}

bool ControlBoardDriver::getTorqueRanges(double* min, double* max)
{
    if (!min)
    {
        yError() << "Error while getting torque ranges: min array is null";
        return false;
    }
    if (!max)
    {
        yError() << "Error while getting torque ranges: max array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getTorqueRange(i, &min[i], &max[i]))
        {
            return false;
        }
    }

    return true;
}

// IEncodersTimed

/**
 * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the
 * future encoders readings
 */
bool ControlBoardDriver::resetEncoder(int j)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while resetting encoder: joint index out of range";
        return false;
    }

    m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).zeroPosition
        = m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).position;

    return true;
}

bool ControlBoardDriver::resetEncoders()
{
    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::resetEncoder(i))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setEncoder(int j, double val)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting encoder: joint index out of range";
        return false;
    }

    m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).zeroPosition
        = m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).position - val;

    return true;
}

bool ControlBoardDriver::setEncoders(const double* vals)
{
    if (!vals)
    {
        yError() << "Error while setting encoders: vals array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::setEncoder(i, vals[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getEncoder(int j, double* v)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!v)
    {
        yError() << "Error while getting encoder: v is null";
        return false;
    }
    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting encoder: joint index out of range";
        return false;
    }

    *v = m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).position
         - m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).zeroPosition;

    return true;
}

bool ControlBoardDriver::getEncoders(double* encs)
{
    if (!encs)
    {
        yError() << "Error while getting encoders: encs array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getEncoder(i, &encs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getEncoderSpeed(int j, double* sp)
{
    // TODO

    return false;
}

bool ControlBoardDriver::getEncoderSpeeds(double* spds)
{
    // TODO

    return false;
}

bool ControlBoardDriver::getEncoderAcceleration(int j, double* spds)
{
    // TODO

    return false;
}

bool ControlBoardDriver::getEncoderAccelerations(double* accs)
{
    // TODO

    return false;
}

bool ControlBoardDriver::getEncoderTimed(int j, double* encs, double* time)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!encs)
    {
        yError() << "Error while getting encoder: encs is null";
        return false;
    }
    if (!time)
    {
        yError() << "Error while getting encoder: time is null";
        return false;
    }
    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting encoder: joint index out of range";
        return false;
    }

    *encs = m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).position
            - m_controlBoardData->joints.at(m_controlBoardData->getJointName(j)).zeroPosition;

    return true;
}

bool ControlBoardDriver::getEncodersTimed(double* encs, double* time)
{
    if (!encs)
    {
        yError() << "Error while getting encoders: encs array is null";
        return false;
    }
    if (!time)
    {
        yError() << "Error while getting encoders: time is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getEncoderTimed(i, &encs[i], &time[i]))
        {
            return false;
        }
    }

    return true;
}

} // namespace gzyarp
} // namespace dev
} // namespace yarp
