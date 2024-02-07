#include "../include/ControlBoardDriver.hh"

#include "../include/ControlBoardData.hh"
#include "../include/ControlBoardDataSingleton.hh"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include <iostream>
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
    yarp::os::Property pluginParameters{};
    pluginParameters.fromString(config.toString().c_str());

    m_controlBoardScopedName = pluginParameters.find(YarpControlBoardScopedName).asString();

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

    *mode = m_controlBoardData->joints.at(axis).interactionMode;

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
        m_controlBoardData->joints.at(axis).interactionMode = mode;
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
        yError() << "Error while getting control mode: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    *mode = m_controlBoardData->joints.at(j).controlMode;

    return true;
}

bool ControlBoardDriver::getControlModes(int* modes)
{
    if (!modes)
    {
        yError() << "Error while getting control modes: modes array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getControlMode(i, &modes[i]))
        {
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

    m_controlBoardData->joints.at(j).controlMode = mode;

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

    name = m_controlBoardData->joints.at(axis).name;

    return true;
}

bool ControlBoardDriver::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    // TODO integrate with IJointCoupled interface

    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;

    return true;
}

// IControlLimits

bool ControlBoardDriver::setLimits(int axis, double min, double max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (axis < 0 || axis >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting limits: axis index out of range";
        return false;
    }

    m_controlBoardData->joints.at(axis).positionLimitMin = min;
    m_controlBoardData->joints.at(axis).positionLimitMax = max;

    return true;
}

bool ControlBoardDriver::getLimits(int axis, double* min, double* max)
{

    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!min)
    {
        yError() << "Error while getting limits: min is null";
        return false;
    }
    if (!max)
    {
        yError() << "Error while getting limits: max is null";
        return false;
    }
    if (axis < 0 || axis >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting limits: axis index out of range";
        return false;
    }

    *min = m_controlBoardData->joints.at(axis).positionLimitMin;
    *max = m_controlBoardData->joints.at(axis).positionLimitMax;

    return true;
}

bool ControlBoardDriver::setVelLimits(int axis, double min, double max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (axis < 0 || axis >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting velocity limits: axis index out of range";
        return false;
    }

    m_controlBoardData->joints.at(axis).velocityLimitMin = min;
    m_controlBoardData->joints.at(axis).velocityLimitMax = max;

    return true;
}

bool ControlBoardDriver::getVelLimits(int axis, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!min)
    {
        yError() << "Error while getting velocity limits: min is null";
        return false;
    }
    if (!max)
    {
        yError() << "Error while getting velocity limits: max is null";
        return false;
    }
    if (axis < 0 || axis >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting velocity limits: axis index out of range";
        return false;
    }

    *min = m_controlBoardData->joints.at(axis).velocityLimitMin;
    *max = m_controlBoardData->joints.at(axis).velocityLimitMax;

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

    *t = m_controlBoardData->joints.at(j).refTorque;

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
        yError() << "Error while setting reference torque: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    m_controlBoardData->joints.at(j).refTorque = t;

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
        yError() << "Error while getting torque: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    *t = m_controlBoardData->joints.at(j).torque;

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
        yError() << "Error while getting torque range: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    *min = -m_controlBoardData->joints.at(j).maxTorqueAbs;
    *max = m_controlBoardData->joints.at(j).maxTorqueAbs;

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

// IPositionControl

bool ControlBoardDriver::positionMove(int j, double ref)
{
    // TODO
    return false;
}

bool ControlBoardDriver::positionMove(const double* refs)
{
    // TODO
    return false;
}
bool ControlBoardDriver::relativeMove(int j, double delta)
{
    // TODO
    return false;
}
bool ControlBoardDriver::relativeMove(const double* deltas)
{
    // TODO
    return false;
}
bool ControlBoardDriver::checkMotionDone(int j, bool* flag)
{
    // TODO
    return false;
}
bool ControlBoardDriver::checkMotionDone(bool* flag)
{
    // TODO
    return false;
}
bool ControlBoardDriver::setRefSpeed(int j, double sp)
{
    // TODO
    return false;
}
bool ControlBoardDriver::setRefSpeeds(const double* spds)
{
    // TODO
    return false;
}
bool ControlBoardDriver::setRefAcceleration(int j, double acc)
{
    // TODO
    return false;
}
bool ControlBoardDriver::setRefAccelerations(const double* accs)
{
    // TODO
    return false;
}
bool ControlBoardDriver::getRefSpeed(int j, double* ref)
{
    // TODO
    return false;
}
bool ControlBoardDriver::getRefSpeeds(double* spds)
{
    // TODO
    return false;
}
bool ControlBoardDriver::getRefAcceleration(int j, double* acc)
{
    // TODO
    return false;
}
bool ControlBoardDriver::getRefAccelerations(double* accs)
{
    // TODO
    return false;
}
bool ControlBoardDriver::stop(int j)
{
    // TODO
    return false;
}
bool ControlBoardDriver::stop()
{
    // TODO
    return false;
}
bool ControlBoardDriver::positionMove(const int n_joint, const int* joints, const double* refs)
{
    // TODO
    return false;
}
bool ControlBoardDriver::relativeMove(const int n_joint, const int* joints, const double* deltas)
{
    // TODO
    return false;
}
bool ControlBoardDriver::checkMotionDone(const int n_joint, const int* joints, bool* flag)
{
    // TODO
    return false;
}
bool ControlBoardDriver::setRefSpeeds(const int n_joint, const int* joints, const double* spds)
{
    // TODO
    return false;
}
bool ControlBoardDriver::setRefAccelerations(const int n_joint,
                                             const int* joints,
                                             const double* accs)
{
    // TODO
    return false;
}
bool ControlBoardDriver::getRefSpeeds(const int n_joint, const int* joints, double* spds)
{
    // TODO
    return false;
}
bool ControlBoardDriver::getRefAccelerations(const int n_joint, const int* joints, double* accs)
{
    // TODO
    return false;
}
bool ControlBoardDriver::stop(const int n_joint, const int* joints)
{
    // TODO
    return false;
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

    m_controlBoardData->joints.at(j).zeroPosition = m_controlBoardData->joints.at(j).position;

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
        yError() << "Error while setting encoder: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    m_controlBoardData->joints.at(j).zeroPosition = m_controlBoardData->joints.at(j).position - val;

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
        yError() << "Error while getting encoder: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    *v = m_controlBoardData->joints.at(j).position - m_controlBoardData->joints.at(j).zeroPosition;

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
        yError() << "Error while getting encoder: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    *encs
        = m_controlBoardData->joints.at(j).position - m_controlBoardData->joints.at(j).zeroPosition;

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
