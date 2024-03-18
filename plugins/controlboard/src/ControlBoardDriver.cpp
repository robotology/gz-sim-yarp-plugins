#include <ControlBoardDriver.hh>

#include <ControlBoardData.hh>
#include <ControlBoardDataSingleton.hh>

#include <cmath>
#include <cstddef>
#include <exception>
#include <mutex>
#include <string>

#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Vocab.h>

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

    m_controlBoardId = pluginParameters.find(YarpControlBoardScopedName).asString();

    m_controlBoardData
        = ::gzyarp::ControlBoardDataSingleton::getControlBoardHandler()->getControlBoardData(
            m_controlBoardId);

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

// IRemoteVariables

bool ControlBoardDriver::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    // TODO
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
    if (!ControlBoardDriver::checkRefTorqueIsValid(t))
    {
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

bool ControlBoardDriver::checkRefTorqueIsValid(double refTorque)
{
    if (std::isnan(refTorque) || std::isinf(refTorque))
    {
        yError() << "Reference torque is not valid.";
        return false;
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

// IPositionDirect

bool ControlBoardDriver::setPosition(int j, double ref)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting position: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    if (m_controlBoardData->joints.at(j).controlMode != VOCAB_CM_POSITION_DIRECT)
    {
        yError() << "Error while setting position: joint " + std::to_string(j)
                        + " is not in position direct mode";
        return false;
    }

    m_controlBoardData->joints.at(j).refPosition = ref;

    return true;
}

bool ControlBoardDriver::setPositions(const int n_joint, const int* joints, const double* refs)
{
    if (!joints)
    {
        yError() << "Error while setting positions: joints array is null";
        return false;
    }
    if (!refs)
    {
        yError() << "Error while setting positions: refs array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setPosition(joints[i], refs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setPositions(const double* refs)
{
    if (!refs)
    {
        yError("Error while setting positions: refs array is null");
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::setPosition(i, refs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getRefPosition(const int joint, double* ref)
{
    if (joint < 0 || joint >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting reference position: joint index " + std::to_string(joint)
                        + " out of range";
        return false;
    }

    *ref = m_controlBoardData->joints.at(joint).refPosition;

    return true;
}

bool ControlBoardDriver::getRefPositions(double* refs)
{
    if (!refs)
    {
        yError() << "Error while getting reference positions: refs array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getRefPosition(i, &refs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    if (!joints)
    {
        yError() << "Error while getting reference positions: joints array is null";
        return false;
    }
    if (!refs)
    {
        yError() << "Error while getting reference positions: refs array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getRefPosition(joints[i], &refs[i]))
        {
            return false;
        }
    }

    return true;
}

// IPositionControl

bool ControlBoardDriver::positionMove(int j, double ref)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting reference position for trajectory generation: joint index "
                        + std::to_string(j) + " out of range";
        return false;
    }

    auto& joint = m_controlBoardData->joints.at(j);

    joint.trajectoryGenerationRefPosition = ref;

    // TODO: use getLimits when recursive mutexes are implemented

    auto limitMin = m_controlBoardData->joints.at(j).positionLimitMin;
    auto limitMax = m_controlBoardData->joints.at(j).positionLimitMax;

    joint.trajectoryGenerator->setLimits(limitMin, limitMax);
    joint.trajectoryGenerator->initTrajectory(joint.position,
                                              joint.trajectoryGenerationRefPosition,
                                              joint.trajectoryGenerationRefSpeed,
                                              joint.trajectoryGenerationRefAcceleration,
                                              m_controlBoardData->controlUpdatePeriod);

    return true;
}

bool ControlBoardDriver::positionMove(const double* refs)
{
    if (!refs)
    {
        yError() << "Error while setting reference positions for trajectory generation: refs array "
                    "is null";
        return false;
    }

    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::positionMove(i, refs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::relativeMove(int j, double delta)
{
    // Check on valid joint number done in setPosition
    return setPosition(j, m_controlBoardData->joints.at(j).position + delta);
}

bool ControlBoardDriver::relativeMove(const double* deltas)
{
    if (!deltas)
    {
        yError() << "Error while setting relative positions: deltas array is null";
        return false;
    }

    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::relativeMove(i, deltas[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::checkMotionDone(int j, bool* flag)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while checking motion done: joint index out of range";
        return false;
    }

    *flag = m_controlBoardData->joints.at(j).isMotionDone;

    return true;
}

bool ControlBoardDriver::checkMotionDone(bool* flag)
{
    if (!flag)
    {
        yError() << "Error while checking motion done: flag is null";
        return false;
    }

    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::checkMotionDone(i, &flag[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setRefSpeed(int j, double sp)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting reference speed: joint index out of range";
        return false;
    }

    m_controlBoardData->joints.at(j).trajectoryGenerationRefSpeed = sp;

    return true;
}

bool ControlBoardDriver::setRefSpeeds(const double* spds)
{
    if (!spds)
    {
        yError() << "Error while setting reference speeds: spds array is null";
        return false;
    }

    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::setRefSpeed(i, spds[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setRefAcceleration(int j, double acc)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while setting reference acceleration: joint index out of range";
        return false;
    }

    m_controlBoardData->joints.at(j).trajectoryGenerationRefAcceleration = acc;

    return true;
}

bool ControlBoardDriver::setRefAccelerations(const double* accs)
{
    if (!accs)
    {
        yError() << "Error while setting reference accelerations: accs array is null";
        return false;
    }

    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::setRefAcceleration(i, accs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getRefSpeed(int j, double* ref)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting reference speed: joint index out of range";
        return false;
    }

    *ref = m_controlBoardData->joints.at(j).trajectoryGenerationRefSpeed;

    return true;
}

bool ControlBoardDriver::getRefSpeeds(double* spds)
{
    if (!spds)
    {
        yError() << "Error while getting reference speeds: spds array is null";
        return false;
    }

    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getRefSpeed(i, &spds[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getRefAcceleration(int j, double* acc)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting reference acceleration: joint index out of range";
        return false;
    }

    *acc = m_controlBoardData->joints.at(j).trajectoryGenerationRefAcceleration;

    return true;
}

bool ControlBoardDriver::getRefAccelerations(double* accs)
{
    if (!accs)
    {
        yError() << "Error while getting reference accelerations: accs array is null";
        return false;
    }

    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getRefAcceleration(i, &accs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::stop(int j)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while stopping: joint index out of range";
        return false;
    }

    switch (m_controlBoardData->joints.at(j).controlMode)
    {
    case VOCAB_CM_POSITION:
        m_controlBoardData->joints.at(j).trajectoryGenerationRefPosition
            = m_controlBoardData->joints.at(j).position;
        m_controlBoardData->joints.at(j).trajectoryGenerator->abortTrajectory(
            m_controlBoardData->joints.at(j).position);
        break;
    case VOCAB_CM_POSITION_DIRECT:
        m_controlBoardData->joints.at(j).trajectoryGenerationRefPosition
            = m_controlBoardData->joints.at(j).position;
        m_controlBoardData->joints.at(j).refPosition = m_controlBoardData->joints.at(j).position;
        break;
    case VOCAB_CM_VELOCITY:
        // TODO velocity control
        yWarning() << "stop not implemented for velocity control mode";
        break;
    case VOCAB_CM_MIXED:
        yWarning() << "stop not implemented for mixed control mode";
        // TODO mixed control
        break;
    }

    return true;
}

bool ControlBoardDriver::stop()
{
    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::stop(i))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::positionMove(const int n_joint, const int* joints, const double* refs)
{
    if (!joints)
    {
        yError() << "Error while setting reference positions for trajectory generation: joints "
                    "array is null";
        return false;
    }
    if (!refs)
    {
        yError() << "Error while setting reference positions for trajectory generation: refs array "
                    "is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::positionMove(joints[i], refs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::relativeMove(const int n_joint, const int* joints, const double* deltas)
{
    if (!joints)
    {
        yError() << "Error while setting relative positions: joints array is null";
        return false;
    }
    if (!deltas)
    {
        yError() << "Error while setting relative positions: deltas array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::relativeMove(joints[i], deltas[i]))
        {
            return false;
        }
    }

    return true;
}
bool ControlBoardDriver::checkMotionDone(const int n_joint, const int* joints, bool* flag)
{
    if (!joints)
    {
        yError() << "Error while checking motion done: joints array is null";
        return false;
    }
    if (!flag)
    {
        yError() << "Error while checking motion done: flag array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::checkMotionDone(joints[i], &flag[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::setRefSpeeds(const int n_joint, const int* joints, const double* spds)
{
    if (!joints)
    {
        yError() << "Error while setting reference speeds: joints array is null";
        return false;
    }
    if (!spds)
    {
        yError() << "Error while setting reference speeds: spds array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setRefSpeed(joints[i], spds[i]))
        {
            return false;
        }
    }

    return true;
}
bool ControlBoardDriver::setRefAccelerations(const int n_joint,
                                             const int* joints,
                                             const double* accs)
{
    if (!joints)
    {
        yError() << "Error while setting reference accelerations: joints array is null";
        return false;
    }
    if (!accs)
    {
        yError() << "Error while setting reference accelerations: accs array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setRefAcceleration(joints[i], accs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getRefSpeeds(const int n_joint, const int* joints, double* spds)
{
    if (!joints)
    {
        yError() << "Error while getting reference speeds: joints array is null";
        return false;
    }
    if (!spds)
    {
        yError() << "Error while getting reference speeds: spds array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getRefSpeed(joints[i], &spds[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getRefAccelerations(const int n_joint, const int* joints, double* accs)
{
    if (!joints)
    {
        yError() << "Error while getting reference accelerations: joints array is null";
        return false;
    }
    if (!accs)
    {
        yError() << "Error while getting reference accelerations: accs array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getRefAcceleration(joints[i], &accs[i]))
        {
            return false;
        }
    }

    return true;
}
bool ControlBoardDriver::stop(const int n_joint, const int* joints)
{
    if (!joints)
    {
        yError() << "Error while stopping: joints array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::stop(joints[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getTargetPosition(const int joint, double* ref)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (joint < 0 || joint >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting target position: joint index " + std::to_string(joint)
                        + " out of range";
        return false;
    }

    *ref = m_controlBoardData->joints.at(joint).trajectoryGenerationRefPosition;

    return true;
}

bool ControlBoardDriver::getTargetPositions(double* refs)
{
    if (!refs)
    {
        yError() << "Error while getting target positions: refs array is null";
        return false;
    }

    for (size_t i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getTargetPosition(i, &refs[i]))
        {
            return false;
        }
    }

    return true;
}

bool ControlBoardDriver::getTargetPositions(const int n_joint, const int* joints, double* refs)
{
    if (!joints)
    {
        yError() << "Error while getting target positions: joints array is null";
        return false;
    }
    if (!refs)
    {
        yError() << "Error while getting target positions: refs array is null";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getTargetPosition(joints[i], &refs[i]))
        {
            return false;
        }
    }

    return true;
}

// IVelocityControl

bool ControlBoardDriver::velocityMove(int j, double sp)
{
    // TODO
    return true;
}
bool ControlBoardDriver::velocityMove(const double* sp)
{
    // TODO
    return true;
}
bool ControlBoardDriver::velocityMove(const int n_joint, const int* joints, const double* spds)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getRefVelocity(const int joint, double* vel)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getRefVelocities(double* vels)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getRefVelocities(const int n_joint, const int* joints, double* vels)
{
    // TODO
    return true;
}

// ICurrentControl

bool ControlBoardDriver::getNumberOfMotors(int* ax)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getCurrent(int m, double* curr)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getCurrents(double* currs)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getCurrentRange(int m, double* min, double* max)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getCurrentRanges(double* min, double* max)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setRefCurrents(const double* currs)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setRefCurrent(int m, double curr)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setRefCurrents(const int n_motor, const int* motors, const double* currs)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getRefCurrents(double* currs)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getRefCurrent(int m, double* curr)
{
    // TODO
    return true;
}

// IPidControl

bool ControlBoardDriver::setPid(const PidControlTypeEnum& pidtype, int j, const Pid& pid)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setPids(const PidControlTypeEnum& pidtype, const Pid* pids)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setPidReference(const PidControlTypeEnum& pidtype, int j, double ref)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setPidReferences(const PidControlTypeEnum& pidtype, const double* refs)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setPidErrorLimits(const PidControlTypeEnum& pidtype, const double* limits)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPidError(const PidControlTypeEnum& pidtype, int j, double* err)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPidErrors(const PidControlTypeEnum& pidtype, double* errs)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPidOutput(const PidControlTypeEnum& pidtype, int j, double* out)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPidOutputs(const PidControlTypeEnum& pidtype, double* outs)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPid(const PidControlTypeEnum& pidtype, int j, Pid* pid)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPids(const PidControlTypeEnum& pidtype, Pid* pids)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPidReference(const PidControlTypeEnum& pidtype, int j, double* ref)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPidReferences(const PidControlTypeEnum& pidtype, double* refs)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double* limit)
{
    // TODO
    return true;
}
bool ControlBoardDriver::getPidErrorLimits(const PidControlTypeEnum& pidtype, double* limits)
{
    // TODO
    return true;
}
bool ControlBoardDriver::resetPid(const PidControlTypeEnum& pidtype, int j)
{
    // TODO
    return true;
}
bool ControlBoardDriver::disablePid(const PidControlTypeEnum& pidtype, int j)
{
    // TODO
    return true;
}
bool ControlBoardDriver::enablePid(const PidControlTypeEnum& pidtype, int j)
{
    // TODO
    return true;
}
bool ControlBoardDriver::setPidOffset(const PidControlTypeEnum& pidtype, int j, double v)
{
    // TODO
    return true;
}
bool ControlBoardDriver::isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    // TODO
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
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!sp)
    {
        yError() << "Error while getting encoder speed: sp is null";
        return false;
    }

    if (j < 0 || j >= m_controlBoardData->joints.size())
    {
        yError() << "Error while getting encoder speed: joint index " + std::to_string(j)
                        + " out of range";
        return false;
    }

    *sp = m_controlBoardData->joints.at(j).velocity;

    return true;
}

bool ControlBoardDriver::getEncoderSpeeds(double* spds)
{
    if (!spds)
    {
        yError() << "Error while getting encoder speeds: spds array is null";
        return false;
    }

    for (int i = 0; i < m_controlBoardData->joints.size(); i++)
    {
        if (!ControlBoardDriver::getEncoderSpeed(i, &spds[i]))
        {
            return false;
        }
    }

    return true;
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
