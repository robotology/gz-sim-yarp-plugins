
#include <ControlBoardData.hh>
#include <gzyarp/Common.hh>

namespace gzyarp
{


bool ControlBoardData::initCoupledJoints() {
    if (this->ijointcoupling) {
        bool ok = this->ijointcoupling->getCoupledActuatedAxes(coupledActuatedAxes);
        ok = ok & this->ijointcoupling->getCoupledPhysicalJoints(coupledPhysicalJoints);
        return ok;
    }
    return true;
}
bool ControlBoardData::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    try
    {
        // If there is coupling let's check if we have to change the interaction mode for all the coupled joints
        if (this->ijointcoupling)
        {
            // If the joint is coupled, we have to change the interaction mode for all the coupled joints
            if(std::find(coupledActuatedAxes.begin(), coupledActuatedAxes.end(), axis) != coupledActuatedAxes.end())
            {
                for (auto& actuatedAxis : coupledActuatedAxes)
                {
                    this->actuatedAxes.at(actuatedAxis).commonJointProperties.interactionMode = mode;
                }
                for (auto& physicalJoint : coupledPhysicalJoints)
                {
                    this->physicalJoints.at(physicalJoint).commonJointProperties.interactionMode = mode;
                }
            } // if the joint is not coupled, we change the interaction mode only for the selected joint
            else
            {
                this->actuatedAxes.at(axis).commonJointProperties.interactionMode = mode;
                this->physicalJoints.at(axis).commonJointProperties.interactionMode = mode;
            }
        } // No coupling, we change the interaction mode only for the selected joint
        else {
            this->physicalJoints.at(axis).commonJointProperties.interactionMode = mode;
            this->actuatedAxes.at(axis).commonJointProperties.interactionMode = mode;
        }
    } catch (const std::exception& e)
    {
        yError() << "Error while setting interaction mode for axis " + std::to_string(axis) + ": \n"
                        + e.what();
        return false;
    }
    return true;
}

bool ControlBoardData::setControlMode(int j, int mode)
{
    int desired_mode = mode;

    if (j < 0 || j >= this->actuatedAxes.size())
    {
        yError() << "Error while setting control mode: joint index out of range";
        return false;
    }

    // Only accept supported control modes
    // The only not supported control mode is
    // (for now) VOCAB_CM_MIXED
    switch (mode)
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_POSITION_DIRECT:
    case VOCAB_CM_VELOCITY:
    case VOCAB_CM_TORQUE:
    case VOCAB_CM_MIXED:
    case VOCAB_CM_PWM:
    case VOCAB_CM_CURRENT:
    case VOCAB_CM_IDLE:
    case VOCAB_CM_FORCE_IDLE:
        break; // no-op
    default:
        yWarning() << "Requested a control mode" << yarp::os::Vocab32::decode(mode)
                   << "that is not supported by"
                   << "gz-sim-yarp-controlboard-system plugin.";
        return false;
    }

    auto& actAxis = this->actuatedAxes.at(j);
    auto& physJoint = this->physicalJoints.at(j);

    if (actAxis.commonJointProperties.controlMode == desired_mode)
    {
        // Nothing to do
        return true;
    }

    // If joint is in hw fault, only a force idle command can recover it
    if (actAxis.commonJointProperties.controlMode == VOCAB_CM_HW_FAULT && mode != VOCAB_CM_FORCE_IDLE)
    {
        return true;
    }

    if (mode == VOCAB_CM_FORCE_IDLE)
    {
        // Clean the fault status and set control mode to idle
        desired_mode = VOCAB_CM_IDLE;
    }

    // If there is coupling let's check if we have to change the control mode for all the coupled joints
    if (this->ijointcoupling)
    {
        // If the joint is coupled, we have to change the control mode for all the coupled joints
        if (std::find(coupledActuatedAxes.begin(), coupledActuatedAxes.end(), j) != coupledActuatedAxes.end())
        {
            for (auto& actuatedAxis : coupledActuatedAxes)
            {
                this->actuatedAxes.at(actuatedAxis).commonJointProperties.controlMode = desired_mode;
            }
            for (auto& physicalJoint : coupledPhysicalJoints)
            {
                this->physicalJoints.at(physicalJoint).commonJointProperties.controlMode = desired_mode;
            }
        } // if the joint is not coupled, we change the control mode only for the selected joint
        else
        {
            actAxis.commonJointProperties.controlMode = desired_mode;
            physJoint.commonJointProperties.controlMode = desired_mode;
        }
    } // No coupling, we change the control mode only for the selected joint
    else {
        physJoint.commonJointProperties.controlMode = desired_mode;
        actAxis.commonJointProperties.controlMode = desired_mode;
    }

    // Reset reference values when switching control mode
    actAxis.commonJointProperties.refPosition = actAxis.commonJointProperties.position;
    actAxis.commonJointProperties.refVelocity = 0.0;
    actAxis.commonJointProperties.refTorque = 0.0;
    actAxis.trajectoryGenerationRefPosition = actAxis.commonJointProperties.position;

    if (desired_mode == VOCAB_CM_POSITION)
    {
        actAxis.trajectoryGenerator->initTrajectory(
            actAxis.commonJointProperties.position,
            actAxis.trajectoryGenerationRefPosition,
            actAxis.trajectoryGenerationRefSpeed,
            actAxis.trajectoryGenerationRefAcceleration,
            this->controlUpdatePeriod);
    }

    return true;
}

double ControlBoardData::convertGazeboGainToUserGain(PhysicalJointProperties& joint, double value)
{
    // TODO discriminate between joint types
    return gzyarp::convertRadianGainToDegreeGains(value);
}

double ControlBoardData::convertGazeboToUser(PhysicalJointProperties& joint, double value)
{
    // TODO discriminate between joint types
    return gzyarp::convertRadiansToDegrees(value);
}

double ControlBoardData::convertUserToGazebo(PhysicalJointProperties& joint, double value)
{
    // TODO discriminate between joint types
    return gzyarp::convertDegreesToRadians(value);
}

double ControlBoardData::convertUserGainToGazeboGain(PhysicalJointProperties& joint, double value)
{
    // TODO discriminate between joint types
    return gzyarp::convertDegreeGainToRadianGains(value);
}

} // namespace gzyarp
