#pragma once

#include "ControlBoardData.hh"

#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PidEnums.h>
#include <yarp/os/Searchable.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{

const std::string YarpControlBoardScopedName = "robotScopedName";

class ControlBoardDriver : public DeviceDriver,
                           public IAxisInfo,
                           public IEncodersTimed,
                           public IInteractionMode,
                           public IControlMode,
                           public IControlLimits,
                           public IRemoteVariables,
                           public ITorqueControl,
                           public IPositionDirect,
                           public IPositionControl,
                           public IVelocityControl,
                           public ICurrentControl,
                           public IPidControl
{
public:
    // DeviceDriver

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IAxisInfo

    bool getAxisName(int axis, std::string& name) override;
    bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

    // IEncodersTimed

    bool resetEncoder(int j) override;
    bool resetEncoders() override;
    bool setEncoder(int j, double val) override;
    bool setEncoders(const double* vals) override;
    bool getEncoder(int j, double* v) override;
    bool getEncoders(double* encs) override;
    bool getEncoderSpeed(int j, double* sp) override;
    bool getEncoderSpeeds(double* spds) override;
    bool getEncoderAcceleration(int j, double* spds) override;
    bool getEncoderAccelerations(double* accs) override;
    bool getEncodersTimed(double* encs, double* time) override;
    bool getEncoderTimed(int j, double* encs, double* time) override;

    // IInteractionMode

    bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode) override;
    bool
    getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes) override;
    bool getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
    bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    bool
    setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes) override;
    bool setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

    // IControlMode

    bool getControlMode(int j, int* mode) override;
    bool getControlModes(int* modes) override;
    bool getControlModes(const int n_joint, const int* joints, int* modes) override;
    bool setControlMode(const int j, const int mode) override;
    bool setControlModes(const int n_joint, const int* joints, int* modes) override;
    bool setControlModes(int* modes) override;

    // IControlLimits

    bool setLimits(int axis, double min, double max) override;
    bool getLimits(int axis, double* min, double* max) override;
    bool setVelLimits(int axis, double min, double max) override;
    bool getVelLimits(int axis, double* min, double* max) override;

    // IRemoteVariables

    bool getRemoteVariable(std::string key, yarp::os::Bottle& val) override;
    bool setRemoteVariable(std::string key, const yarp::os::Bottle& val) override;
    bool getRemoteVariablesList(yarp::os::Bottle* listOfKeys) override;

    // ITorqueControl

    bool getAxes(int* ax) override;
    bool getRefTorques(double* t) override;
    bool getRefTorque(int j, double* t) override;
    bool setRefTorques(const double* t) override;
    bool setRefTorque(int j, double t) override;
    bool setRefTorques(const int n_joint, const int* joints, const double* t) override;
    bool getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params) override;
    bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;
    bool getTorque(int j, double* t) override;
    bool getTorques(double* t) override;
    bool getTorqueRange(int j, double* min, double* max) override;
    bool getTorqueRanges(double* min, double* max) override;

    // IPositionDirect

    bool setPosition(int j, double ref) override;
    bool setPositions(const int n_joint, const int* joints, const double* refs) override;
    bool setPositions(const double* refs) override;
    bool getRefPosition(const int joint, double* ref) override;
    bool getRefPositions(double* refs) override;
    bool getRefPositions(const int n_joint, const int* joints, double* refs) override;

    // IPositionControl

    bool positionMove(int j, double ref) override;
    bool positionMove(const double* refs) override;
    bool relativeMove(int j, double delta) override;
    bool relativeMove(const double* deltas) override;
    bool checkMotionDone(int j, bool* flag) override;
    bool checkMotionDone(bool* flag) override;
    bool setRefSpeed(int j, double sp) override;
    bool setRefSpeeds(const double* spds) override;
    bool setRefAcceleration(int j, double acc) override;
    bool setRefAccelerations(const double* accs) override;
    bool getRefSpeed(int j, double* ref) override;
    bool getRefSpeeds(double* spds) override;
    bool getRefAcceleration(int j, double* acc) override;
    bool getRefAccelerations(double* accs) override;
    bool stop(int j) override;
    bool stop() override;
    bool positionMove(const int n_joint, const int* joints, const double* refs) override;
    bool relativeMove(const int n_joint, const int* joints, const double* deltas) override;
    bool checkMotionDone(const int n_joint, const int* joints, bool* flag) override;
    bool setRefSpeeds(const int n_joint, const int* joints, const double* spds) override;
    bool setRefAccelerations(const int n_joint, const int* joints, const double* accs) override;
    bool getRefSpeeds(const int n_joint, const int* joints, double* spds) override;
    bool getRefAccelerations(const int n_joint, const int* joints, double* accs) override;
    bool stop(const int n_joint, const int* joints) override;
    bool getTargetPosition(const int joint, double* ref) override;
    bool getTargetPositions(double* refs) override;
    bool getTargetPositions(const int n_joint, const int* joints, double* refs) override;

    // IVelocityControl

    bool velocityMove(int j, double sp) override;
    bool velocityMove(const double* sp) override;
    bool velocityMove(const int n_joint, const int* joints, const double* spds) override;
    bool getRefVelocity(const int joint, double* vel) override;
    bool getRefVelocities(double* vels) override;
    bool getRefVelocities(const int n_joint, const int* joints, double* vels) override;

    // ICurrentControl

    bool getNumberOfMotors(int* ax) override;
    bool getCurrent(int m, double* curr) override;
    bool getCurrents(double* currs) override;
    bool getCurrentRange(int m, double* min, double* max) override;
    bool getCurrentRanges(double* min, double* max) override;
    bool setRefCurrents(const double* currs) override;
    bool setRefCurrent(int m, double curr) override;
    bool setRefCurrents(const int n_motor, const int* motors, const double* currs) override;
    bool getRefCurrents(double* currs) override;
    bool getRefCurrent(int m, double* curr) override;

    // IPidControl

    bool setPid(const PidControlTypeEnum& pidtype, int j, const Pid& pid) override;
    bool setPids(const PidControlTypeEnum& pidtype, const Pid* pids) override;
    bool setPidReference(const PidControlTypeEnum& pidtype, int j, double ref) override;
    bool setPidReferences(const PidControlTypeEnum& pidtype, const double* refs) override;
    bool setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit) override;
    bool setPidErrorLimits(const PidControlTypeEnum& pidtype, const double* limits) override;
    bool getPidError(const PidControlTypeEnum& pidtype, int j, double* err) override;
    bool getPidErrors(const PidControlTypeEnum& pidtype, double* errs) override;
    bool getPidOutput(const PidControlTypeEnum& pidtype, int j, double* out) override;
    bool getPidOutputs(const PidControlTypeEnum& pidtype, double* outs) override;
    bool getPid(const PidControlTypeEnum& pidtype, int j, Pid* pid) override;
    bool getPids(const PidControlTypeEnum& pidtype, Pid* pids) override;
    bool getPidReference(const PidControlTypeEnum& pidtype, int j, double* ref) override;
    bool getPidReferences(const PidControlTypeEnum& pidtype, double* refs) override;
    bool getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double* limit) override;
    bool getPidErrorLimits(const PidControlTypeEnum& pidtype, double* limits) override;
    bool resetPid(const PidControlTypeEnum& pidtype, int j) override;
    bool disablePid(const PidControlTypeEnum& pidtype, int j) override;
    bool enablePid(const PidControlTypeEnum& pidtype, int j) override;
    bool setPidOffset(const PidControlTypeEnum& pidtype, int j, double v) override;
    bool isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) override;

private:
    std::string m_controlBoardScopedName;
    ControlBoardData* m_controlBoardData;

    bool checkRefTorqueIsValid(double refTorque);
};

} // namespace gzyarp
} // namespace dev
} // namespace yarp
