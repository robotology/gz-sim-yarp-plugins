#include <ControlBoardDriver.hh>

#include <ControlBoardData.hh>

#include <cmath>
#include <cstddef>
#include <exception>
#include <mutex>
#include <string>

#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PidEnums.h>
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

    return true;
}

bool ControlBoardDriver::close()
{
    return true;
}

// IInteractionMode

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    *mode = m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.interactionMode;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getInteractionModes(int n_joints,
                                             int* joints,
                                             yarp::dev::InteractionModeEnum* modes)
{
    if (!joints)
    {
        yError() << "Error while getting interaction modes: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!modes)
    {
        yError() << "Error while getting interaction modes: modes array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joints; i++)
    {
        if (!ControlBoardDriver::getInteractionMode(joints[i], &modes[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (!modes)
    {
        yError() << "Error while getting interaction modes: modes array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getInteractionMode(i, &modes[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);
    return m_controlBoardData->setInteractionMode(axis, mode) ? YARP_DEV_RETURN_VALUE_OK_CH40 : YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setInteractionModes(int n_joints,
                                             int* joints,
                                             yarp::dev::InteractionModeEnum* modes)
{
    if (!joints)
    {
        yError() << "Error while setting interaction modes: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!modes)
    {
        yError() << "Error while setting interaction modes: modes array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joints; i++)
    {
        if (!ControlBoardDriver::setInteractionMode(joints[i], modes[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (!modes)
    {
        yError() << "Error while setting interaction modes: modes array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::setInteractionMode(i, modes[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

// IControlMode

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getControlMode(int j, int* mode)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!mode)
    {
        yError() << "Error while getting control mode: mode array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting control mode: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *mode = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.controlMode;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getControlModes(int* modes)
{
    if (!modes)
    {
        yError() << "Error while getting control modes: modes array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getControlMode(i, &modes[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getControlModes(const int n_joint, const int* joints, int* modes)
{
    if (!joints)
    {
        yError() << "Error while getting control modes: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!modes)
    {
        yError() << "Error while getting control modes: modes array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getControlMode(joints[i], &modes[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setControlMode(const int j, const int mode)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);
    return m_controlBoardData->setControlMode(j, mode) ? YARP_DEV_RETURN_VALUE_OK_CH40 : YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setControlModes(const int n_joint, const int* joints, int* modes)
{
    if (!joints)
    {
        yError() << "Error while setting control modes: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!modes)
    {
        yError() << "Error while setting control modes: modes array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setControlMode(joints[i], modes[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setControlModes(int* modes)
{
    if (!modes)
    {
        yError() << "Error while setting control modes: modes array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::setControlMode(i, modes[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

// IAxisInfo

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getAxisName(int axis, std::string& name)
{
    // TODO integrate with IJointCoupled interface

    name = m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.name;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    // TODO integrate with IJointCoupled interface

    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

// IControlLimits

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPosLimits(int axis, double min, double max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (axis < 0 || axis >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while setting limits: axis index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.positionLimitMin = min;
    m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.positionLimitMax = max;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPosLimits(int axis, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!min)
    {
        yError() << "Error while getting limits: min is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!max)
    {
        yError() << "Error while getting limits: max is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (axis < 0 || axis >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting limits: axis index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *min = m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.positionLimitMin;
    *max = m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.positionLimitMax;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setVelLimits(int axis, double min, double max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (axis < 0 || axis >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while setting velocity limits: axis index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.velocityLimitMin = min;
    m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.velocityLimitMax = max;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getVelLimits(int axis, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!min)
    {
        yError() << "Error while getting velocity limits: min is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!max)
    {
        yError() << "Error while getting velocity limits: max is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (axis < 0 || axis >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting velocity limits: axis index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *min = m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.velocityLimitMin;
    *max = m_controlBoardData->actuatedAxes.at(axis).commonJointProperties.velocityLimitMax;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

// IRemoteVariables

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

// ITorqueControl

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getAxes(int* ax)
{
    // TODO integrate with IJointCoupled interface

    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);
    *ax = m_controlBoardData->actuatedAxes.size();

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRefTorque(int j, double* t)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!t)
    {
        yError() << "Error while getting reference torque: t is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting reference torque: joint index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *t = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.refTorque;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRefTorques(double* t)
{
    if (!t)
    {
        yError() << "Error while getting reference torques: t array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getRefTorque(i, &t[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setRefTorque(int j, double t)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while setting reference torque: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!ControlBoardDriver::checkRefTorqueIsValid(t))
    {
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    m_controlBoardData->actuatedAxes.at(j).commonJointProperties.refTorque = t;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setRefTorques(const double* t)
{
    if (!t)
    {
        yError() << "Error while setting reference torques: t array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::setRefTorque(i, t[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setRefTorques(const int n_joint, const int* joints, const double* t)
{
    if (!joints)
    {
        yError() << "Error while setting reference torques: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!t)
    {
        yError() << "Error while setting reference torques: t array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setRefTorque(joints[i], t[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
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

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params)
{
    // TODO

    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    // TODO

    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTorque(int j, double* t)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!t)
    {
        yError() << "Error while getting torque: t is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting torque: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *t = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.torque;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTorques(double* t)
{
    if (!t)
    {
        yError() << "Error while getting torques: t array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getTorque(i, &t[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTorqueRange(int j, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!min)
    {
        yError() << "Error while getting torque range: min is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!max)
    {
        yError() << "Error while getting torque range: max is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting torque range: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *min = -m_controlBoardData->actuatedAxes.at(j).commonJointProperties.maxTorqueAbs;
    *max = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.maxTorqueAbs;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTorqueRanges(double* min, double* max)
{
    if (!min)
    {
        yError() << "Error while getting torque ranges: min array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!max)
    {
        yError() << "Error while getting torque ranges: max array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getTorqueRange(i, &min[i], &max[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

// IPositionDirect

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPosition(int j, double ref)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while setting position: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    if (m_controlBoardData->actuatedAxes.at(j).commonJointProperties.controlMode != VOCAB_CM_POSITION_DIRECT)
    {
        yError() << "Error while setting position: joint " + std::to_string(j)
                        + " is not in position direct mode";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    m_controlBoardData->actuatedAxes.at(j).commonJointProperties.refPosition = ref;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPositions(const int n_joint, const int* joints, const double* refs)
{
    if (!joints)
    {
        yError() << "Error while setting positions: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!refs)
    {
        yError() << "Error while setting positions: refs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setPosition(joints[i], refs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPositions(const double* refs)
{
    if (!refs)
    {
        yError("Error while setting positions: refs array is null");
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::setPosition(i, refs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRefPosition(const int joint, double* ref)
{
    if (joint < 0 || joint >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting reference position: joint index " + std::to_string(joint)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *ref = m_controlBoardData->actuatedAxes.at(joint).commonJointProperties.refPosition;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRefPositions(double* refs)
{
    if (!refs)
    {
        yError() << "Error while getting reference positions: refs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getRefPosition(i, &refs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    if (!joints)
    {
        yError() << "Error while getting reference positions: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!refs)
    {
        yError() << "Error while getting reference positions: refs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getRefPosition(joints[i], &refs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

// IPositionControl

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::positionMove(int j, double ref)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while setting reference position for trajectory generation: joint index "
                        + std::to_string(j) + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    auto& joint = m_controlBoardData->actuatedAxes.at(j);

    joint.trajectoryGenerationRefPosition = ref;

    // TODO: use getLimits when recursive mutexes are implemented

    auto limitMin = joint.commonJointProperties.positionLimitMin;
    auto limitMax = joint.commonJointProperties.positionLimitMax;

    joint.trajectoryGenerator->setLimits(limitMin, limitMax);
    joint.trajectoryGenerator->initTrajectory(joint.commonJointProperties.position,
                                              joint.trajectoryGenerationRefPosition,
                                              joint.trajectoryGenerationRefSpeed,
                                              joint.trajectoryGenerationRefAcceleration,
                                              m_controlBoardData->controlUpdatePeriod);

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::positionMove(const double* refs)
{
    if (!refs)
    {
        yError() << "Error while setting reference positions for trajectory generation: refs array "
                    "is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::positionMove(i, refs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::relativeMove(int j, double delta)
{
    // Check on valid joint number done in setPosition
    return setPosition(j, m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position + delta);
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::relativeMove(const double* deltas)
{
    if (!deltas)
    {
        yError() << "Error while setting relative positions: deltas array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::relativeMove(i, deltas[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::checkMotionDone(int j, bool* flag)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while checking motion done: joint index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *flag = m_controlBoardData->actuatedAxes.at(j).isMotionDone;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::checkMotionDone(bool* flag)
{
    if (!flag)
    {
        yError() << "Error while checking motion done: flag is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::checkMotionDone(i, &flag[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}


YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setTrajSpeed(int j, double sp)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while setting reference speed: joint index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    m_controlBoardData->actuatedAxes.at(j).trajectoryGenerationRefSpeed = sp;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setTrajSpeeds(const double* spds)
{
    if (!spds)
    {
        yError() << "Error while setting reference speeds: spds array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::setTrajSpeed(i, spds[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setTrajAcceleration(int j, double acc)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while setting reference acceleration: joint index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    m_controlBoardData->actuatedAxes.at(j).trajectoryGenerationRefAcceleration = acc;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setTrajAccelerations(const double* accs)
{
    if (!accs)
    {
        yError() << "Error while setting reference accelerations: accs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::setTrajAcceleration(i, accs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTrajSpeed(int j, double* ref)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting reference speed: joint index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *ref = m_controlBoardData->actuatedAxes.at(j).trajectoryGenerationRefSpeed;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTrajSpeeds(double* spds)
{
    if (!spds)
    {
        yError() << "Error while getting reference speeds: spds array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getTrajSpeed(i, &spds[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTrajAcceleration(int j, double* acc)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting reference acceleration: joint index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *acc = m_controlBoardData->actuatedAxes.at(j).trajectoryGenerationRefAcceleration;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTrajAccelerations(double* accs)
{
    if (!accs)
    {
        yError() << "Error while getting reference accelerations: accs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getTrajAcceleration(i, &accs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::stop(int j)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while stopping: joint index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    switch (m_controlBoardData->actuatedAxes.at(j).commonJointProperties.controlMode)
    {
    case VOCAB_CM_POSITION:
        m_controlBoardData->actuatedAxes.at(j).trajectoryGenerationRefPosition
            = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position;
        m_controlBoardData->actuatedAxes.at(j).trajectoryGenerator->abortTrajectory(
            m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position);
        break;
    case VOCAB_CM_POSITION_DIRECT:
        m_controlBoardData->actuatedAxes.at(j).trajectoryGenerationRefPosition
            = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position;
        m_controlBoardData->actuatedAxes.at(j).commonJointProperties.refPosition = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position;
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

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::stop()
{
    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::stop(i))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::positionMove(const int n_joint, const int* joints, const double* refs)
{
    if (!joints)
    {
        yError() << "Error while setting reference positions for trajectory generation: joints "
                    "array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!refs)
    {
        yError() << "Error while setting reference positions for trajectory generation: refs array "
                    "is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::positionMove(joints[i], refs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::relativeMove(const int n_joint, const int* joints, const double* deltas)
{
    if (!joints)
    {
        yError() << "Error while setting relative positions: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!deltas)
    {
        yError() << "Error while setting relative positions: deltas array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::relativeMove(joints[i], deltas[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::checkMotionDone(const int n_joint, const int* joints, bool* flag)
{
    if (!joints)
    {
        yError() << "Error while checking motion done: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!flag)
    {
        yError() << "Error while checking motion done: flag array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::checkMotionDone(joints[i], &flag[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setTrajSpeeds(const int n_joint, const int* joints, const double* spds)
{
    if (!joints)
    {
        yError() << "Error while setting reference speeds: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!spds)
    {
        yError() << "Error while setting reference speeds: spds array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setTrajSpeed(joints[i], spds[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setTrajAccelerations(const int n_joint,
                                             const int* joints,
                                             const double* accs)
{
    if (!joints)
    {
        yError() << "Error while setting reference accelerations: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!accs)
    {
        yError() << "Error while setting reference accelerations: accs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::setTrajAcceleration(joints[i], accs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTrajSpeeds(const int n_joint, const int* joints, double* spds)
{
    if (!joints)
    {
        yError() << "Error while getting reference speeds: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!spds)
    {
        yError() << "Error while getting reference speeds: spds array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getTrajSpeed(joints[i], &spds[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTrajAccelerations(const int n_joint, const int* joints, double* accs)
{
    if (!joints)
    {
        yError() << "Error while getting reference accelerations: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!accs)
    {
        yError() << "Error while getting reference accelerations: accs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getTrajAcceleration(joints[i], &accs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::stop(const int n_joint, const int* joints)
{
    if (!joints)
    {
        yError() << "Error while stopping: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::stop(joints[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTargetPosition(const int joint, double* ref)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (joint < 0 || joint >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting target position: joint index " + std::to_string(joint)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *ref = m_controlBoardData->actuatedAxes.at(joint).trajectoryGenerationRefPosition;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTargetPositions(double* refs)
{
    if (!refs)
    {
        yError() << "Error while getting target positions: refs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (size_t i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getTargetPosition(i, &refs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTargetPositions(const int n_joint, const int* joints, double* refs)
{
    if (!joints)
    {
        yError() << "Error while getting target positions: joints array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!refs)
    {
        yError() << "Error while getting target positions: refs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < n_joint; i++)
    {
        if (!ControlBoardDriver::getTargetPosition(joints[i], &refs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

// IVelocityControl

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::velocityMove(int j, double sp)
{
    size_t numberOfJoints = m_controlBoardData->actuatedAxes.size();
    if (j >= 0 && static_cast<size_t>(j) < numberOfJoints)
    {
        m_controlBoardData->actuatedAxes[j].velocityWatchdog->reset();
        if (m_controlBoardData->actuatedAxes[j].speedRampHandler)
        {
            double refacc = m_controlBoardData->actuatedAxes[j].trajectoryGenerationRefAcceleration;
            double refvel = sp;
            m_controlBoardData->actuatedAxes[j].speedRampHandler->setReference(refvel, refacc);
        }
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::velocityMove(const double* sp)
{
    if (!sp) {return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;}

    size_t numberOfJoints = m_controlBoardData->actuatedAxes.size();
    for (size_t i = 0; i < numberOfJoints; ++i)
    {
        velocityMove(i, sp[i]);
    }
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::velocityMove(const int n_joint, const int* joints, const double* spds)
{
    if (!joints || !spds) {return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;}

    YARP_DEV_RETURN_VALUE_TYPE_CH40 ret = YARP_DEV_RETURN_VALUE_OK_CH40;
    for (int i = 0; i < n_joint && ret; i++)
    {
        ret = velocityMove(joints[i], spds[i]);
    }
    return ret;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTargetVelocity(const int joint, double* vel)
{
    size_t numberOfJoints = m_controlBoardData->actuatedAxes.size();
    if (vel && joint >= 0 && static_cast<size_t>(joint) < numberOfJoints)
    {
        *vel = m_controlBoardData->actuatedAxes[joint].commonJointProperties.refVelocity;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTargetVelocities(double* vels)
{
    if (!vels) {return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;}

    size_t numberOfJoints = m_controlBoardData->actuatedAxes.size();
    YARP_DEV_RETURN_VALUE_TYPE_CH40 ret = YARP_DEV_RETURN_VALUE_OK_CH40;
    for (size_t i = 0; i < numberOfJoints && ret; i++)
    {
        ret = getTargetVelocity(i, &vels[i]);
    }
    return ret;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getTargetVelocities(const int n_joint, const int* joints, double* vels)
{
    if (!joints || !vels) {return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;}

    YARP_DEV_RETURN_VALUE_TYPE_CH40 ret = YARP_DEV_RETURN_VALUE_OK_CH40;
    for (int i = 0; i < n_joint && ret; i++)
    {
        ret = getTargetVelocity(joints[i], &vels[i]);
    }
    return ret;
}

// ICurrentControl

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getNumberOfMotors(int* ax)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getCurrent(int m, double* curr)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getCurrents(double* currs)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getCurrentRange(int m, double* min, double* max)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getCurrentRanges(double* min, double* max)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setRefCurrents(const double* currs)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setRefCurrent(int m, double curr)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setRefCurrents(const int n_motor, const int* motors, const double* currs)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRefCurrents(double* currs)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getRefCurrent(int m, double* curr)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

// IPidControl

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPid(const PidControlTypeEnum& pidtype, int j, const Pid& pid)
{
    gz::math::PID& gzpid = m_controlBoardData->physicalJoints[j].pidControllers[pidtype];
    gzpid.SetPGain(pid.kp);
    gzpid.SetIGain(pid.ki);
    gzpid.SetDGain(pid.kd);

    gzpid.SetPGain(m_controlBoardData->convertUserGainToGazeboGain(m_controlBoardData->physicalJoints[j], pid.kp) / pow(2, pid.scale));
    gzpid.SetIGain(m_controlBoardData->convertUserGainToGazeboGain(m_controlBoardData->physicalJoints[j], pid.ki) / pow(2, pid.scale));
    gzpid.SetDGain(m_controlBoardData->convertUserGainToGazeboGain(m_controlBoardData->physicalJoints[j], pid.kd) / pow(2, pid.scale));

    gzpid.SetIMax(pid.max_int);
    gzpid.SetIMin(-pid.max_int);
    gzpid.SetCmdMax(pid.max_output);
    gzpid.SetCmdMin(-pid.max_output);

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPids(const PidControlTypeEnum& pidtype, const Pid* pids)
{
    size_t i = 0;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 ret = YARP_DEV_RETURN_VALUE_OK_CH40;
    if (!pids) {return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;}
    for (auto it = m_controlBoardData->physicalJoints.begin();
        it != m_controlBoardData->physicalJoints.end();
        it++)
    {
        ret &= setPid(pidtype, i, pids[i]);
    }
    return ret;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPidReference(const PidControlTypeEnum& pidtype, int j, double ref)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPidReferences(const PidControlTypeEnum& pidtype, const double* refs)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPidErrorLimits(const PidControlTypeEnum& pidtype, const double* limits)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidError(const PidControlTypeEnum& pidtype, int j, double* err)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidErrors(const PidControlTypeEnum& pidtype, double* errs)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidOutput(const PidControlTypeEnum& pidtype, int j, double* out)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidOutputs(const PidControlTypeEnum& pidtype, double* outs)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPid(const PidControlTypeEnum& pidtype, int j, Pid* pid)
{
    if (!pid) {return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;}
    gz::math::PID gzpid = m_controlBoardData->physicalJoints[j].pidControllers[pidtype];

    pid->scale = 0;
    pid->kp = m_controlBoardData->convertGazeboGainToUserGain(m_controlBoardData->physicalJoints[j], gzpid.PGain()) * pow(2, pid->scale);
    pid->ki = m_controlBoardData->convertGazeboGainToUserGain(m_controlBoardData->physicalJoints[j], gzpid.IGain()) * pow(2, pid->scale);
    pid->kd = m_controlBoardData->convertGazeboGainToUserGain(m_controlBoardData->physicalJoints[j], gzpid.DGain()) * pow(2, pid->scale);
    pid->max_int = gzpid.IMax();
    pid->max_output = gzpid.CmdMax();

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPids(const PidControlTypeEnum& pidtype, Pid* pids)
{
    size_t i = 0;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 ret = YARP_DEV_RETURN_VALUE_OK_CH40;
    if (!pids) {return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;}
    for (auto it = m_controlBoardData->physicalJoints.begin();
        it != m_controlBoardData->physicalJoints.end();
        it++)
    {
        ret &= getPid(pidtype, i, &pids[i]);
        i++;
    }
    return ret;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidReference(const PidControlTypeEnum& pidtype, int j, double* ref)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidReferences(const PidControlTypeEnum& pidtype, double* refs)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double* limit)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidErrorLimits(const PidControlTypeEnum& pidtype, double* limits)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::resetPid(const PidControlTypeEnum& pidtype, int j)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::disablePid(const PidControlTypeEnum& pidtype, int j)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::enablePid(const PidControlTypeEnum& pidtype, int j)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPidOffset(const PidControlTypeEnum& pidtype, int j, double v)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setPidFeedforward(const PidControlTypeEnum& pidtype, int j, double v)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidOffset(const PidControlTypeEnum& pidtype, int j, double& v)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidFeedforward(const PidControlTypeEnum& pidtype, int j, double& v)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidExtraInfo(const PidControlTypeEnum& pidtype, int j, yarp::dev::PidExtraInfo& info)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getPidExtraInfos(const PidControlTypeEnum& pidtype, std::vector<yarp::dev::PidExtraInfo>& info)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool& enabled)
{
    // TODO
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

// IEncodersTimed



/**
 * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the
 * future encoders readings
 */
YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::resetEncoder(int j)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while resetting encoder: joint index out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    m_controlBoardData->actuatedAxes.at(j).commonJointProperties.zeroPosition = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::resetEncoders()
{
    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::resetEncoder(i))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setEncoder(int j, double val)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while setting encoder: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    m_controlBoardData->actuatedAxes.at(j).commonJointProperties.zeroPosition = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position - val;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::setEncoders(const double* vals)
{
    if (!vals)
    {
        yError() << "Error while setting encoders: vals array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::setEncoder(i, vals[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getEncoder(int j, double* v)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!v)
    {
        yError() << "Error while getting encoder: v is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting encoder: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *v = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position - m_controlBoardData->actuatedAxes.at(j).commonJointProperties.zeroPosition;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getEncoders(double* encs)
{
    if (!encs)
    {
        yError() << "Error while getting encoders: encs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getEncoder(i, &encs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getEncoderSpeed(int j, double* sp)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!sp)
    {
        yError() << "Error while getting encoder speed: sp is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting encoder speed: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *sp = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.velocity;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getEncoderSpeeds(double* spds)
{
    if (!spds)
    {
        yError() << "Error while getting encoder speeds: spds array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getEncoderSpeed(i, &spds[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getEncoderAcceleration(int j, double* acc)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!acc)
    {
        yError() << "Error while getting encoder acceleration: acc is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting encoder acceleration: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *acc = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.acceleration;

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getEncoderAccelerations(double* accs)
{
    if (!accs)
    {
        yError() << "Error while getting encoder accelerations: accs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getEncoderAcceleration(i, &accs[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getEncoderTimed(int j, double* encs, double* time)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData->mutex);

    if (!encs)
    {
        yError() << "Error while getting encoder: encs is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!time)
    {
        yError() << "Error while getting encoder: time is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (j < 0 || j >= m_controlBoardData->actuatedAxes.size())
    {
        yError() << "Error while getting encoder: joint index " + std::to_string(j)
                        + " out of range";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    *encs
        = m_controlBoardData->actuatedAxes.at(j).commonJointProperties.position - m_controlBoardData->actuatedAxes.at(j).commonJointProperties.zeroPosition;
    *time = m_controlBoardData->simTime.getTime();

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 ControlBoardDriver::getEncodersTimed(double* encs, double* time)
{
    if (!encs)
    {
        yError() << "Error while getting encoders: encs array is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }
    if (!time)
    {
        yError() << "Error while getting encoders: time is null";
        return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
    }

    for (int i = 0; i < m_controlBoardData->actuatedAxes.size(); i++)
    {
        if (!ControlBoardDriver::getEncoderTimed(i, &encs[i], &time[i]))
        {
            return YARP_DEV_RETURN_VALUE_ERROR_METHOD_FAILED_CH40;
        }
    }

    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

// IControlBoardData

void ControlBoardDriver::setControlBoardData(::gzyarp::ControlBoardData* controlBoardData)
{
    m_controlBoardData = controlBoardData;
}

} // namespace gzyarp
} // namespace dev
} // namespace yarp
