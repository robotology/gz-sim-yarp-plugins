#pragma once

#include <ControlBoardData.hh>

#include <string>
#include <vector>

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
#include <gzyarp/YarpDevReturnValueCompat.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{

const std::string YarpControlBoardScopedName = "controlBoardId";

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
#if (YARP_VERSION_MAJOR > 3)
                           public IVelocityDirect,
#endif
                           public IVelocityControl,
                           public ICurrentControl,
                           public IPidControl,
                           public ::gzyarp::IControlBoardData
{
public:
    // DeviceDriver

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IAxisInfo

    YARP_DEV_RETURN_VALUE_TYPE_CH40 getAxisName(int axis, std::string& name) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

    // IEncodersTimed

    YARP_DEV_RETURN_VALUE_TYPE_CH40 resetEncoder(int j) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 resetEncoders() override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setEncoder(int j, double val) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setEncoders(const double* vals) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getEncoder(int j, double* v) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getEncoders(double* encs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getEncoderSpeed(int j, double* sp) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getEncoderSpeeds(double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getEncoderAcceleration(int j, double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getEncoderAccelerations(double* accs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getEncodersTimed(double* encs, double* time) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getEncoderTimed(int j, double* encs, double* time) override;

    // IInteractionMode

    YARP_DEV_RETURN_VALUE_TYPE_CH40 getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

    // IControlMode

    YARP_DEV_RETURN_VALUE_TYPE_CH40 getControlMode(int j, int* mode) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getControlModes(int* modes) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getControlModes(const int n_joint, const int* joints, int* modes) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setControlMode(const int j, const int mode) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setControlModes(const int n_joint, const int* joints, int* modes) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setControlModes(int* modes) override;

    // IControlLimits

#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPosLimits(int axis, double min, double max) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPosLimits(int axis, double* min, double* max) override;
#else
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setLimits(int axis, double min, double max) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getLimits(int axis, double* min, double* max) override;
#endif
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setVelLimits(int axis, double min, double max) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getVelLimits(int axis, double* min, double* max) override;

    // IRemoteVariables

    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRemoteVariable(std::string key, yarp::os::Bottle& val) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRemoteVariable(std::string key, const yarp::os::Bottle& val) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRemoteVariablesList(yarp::os::Bottle* listOfKeys) override;

    // ITorqueControl

    YARP_DEV_RETURN_VALUE_TYPE_CH40 getAxes(int* ax) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefTorques(double* t) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefTorque(int j, double* t) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefTorques(const double* t) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefTorque(int j, double t) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefTorques(const int n_joint, const int* joints, const double* t) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTorque(int j, double* t) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTorques(double* t) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTorqueRange(int j, double* min, double* max) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTorqueRanges(double* min, double* max) override;

#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    // IVelocityDirect
    yarp::dev::ReturnValue setRefVelocity(int jnt, double vel) override;
    yarp::dev::ReturnValue setRefVelocity(const std::vector<double>& vels) override;
    yarp::dev::ReturnValue setRefVelocity(const std::vector<int>& jnts, const std::vector<double>& vels) override;
    yarp::dev::ReturnValue getRefVelocity(const int jnt, double& vel) override;
    yarp::dev::ReturnValue getRefVelocity(std::vector<double>& vels) override;
    yarp::dev::ReturnValue getRefVelocity(const std::vector<int>& jnts, std::vector<double>& vels) override;
#endif

    // IPositionDirect

    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPosition(int j, double ref) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPositions(const int n_joint, const int* joints, const double* refs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPositions(const double* refs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefPosition(const int joint, double* ref) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefPositions(double* refs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefPositions(const int n_joint, const int* joints, double* refs) override;

    // IPositionControl

    YARP_DEV_RETURN_VALUE_TYPE_CH40 positionMove(int j, double ref) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 positionMove(const double* refs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 relativeMove(int j, double delta) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 relativeMove(const double* deltas) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 checkMotionDone(int j, bool* flag) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 checkMotionDone(bool* flag) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 stop(int j) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 stop() override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 positionMove(const int n_joint, const int* joints, const double* refs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 relativeMove(const int n_joint, const int* joints, const double* deltas) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 checkMotionDone(const int n_joint, const int* joints, bool* flag) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 stop(const int n_joint, const int* joints) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTargetPosition(const int joint, double* ref) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTargetPositions(double* refs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTargetPositions(const int n_joint, const int* joints, double* refs) override;
#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setTrajSpeed(int j, double sp) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setTrajSpeeds(const double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setTrajAcceleration(int j, double acc) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setTrajAccelerations(const double* accs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTrajSpeed(int j, double* ref) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTrajSpeeds(double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTrajAcceleration(int j, double* acc) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTrajAccelerations(double* accs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setTrajSpeeds(const int n_joint, const int* joints, const double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setTrajAccelerations(const int n_joint, const int* joints, const double* accs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTrajSpeeds(const int n_joint, const int* joints, double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTrajAccelerations(const int n_joint, const int* joints, double* accs) override;
#else
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefSpeed(int j, double sp) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefSpeeds(const double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefAcceleration(int j, double acc) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefAccelerations(const double* accs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefSpeed(int j, double* ref) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefSpeeds(double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefAcceleration(int j, double* acc) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefAccelerations(double* accs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefSpeeds(const int n_joint, const int* joints, const double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefAccelerations(const int n_joint, const int* joints, const double* accs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefSpeeds(const int n_joint, const int* joints, double* spds) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefAccelerations(const int n_joint, const int* joints, double* accs) override;
#endif

    // IVelocityControl

    YARP_DEV_RETURN_VALUE_TYPE_CH40 velocityMove(int j, double sp) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 velocityMove(const double* sp) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 velocityMove(const int n_joint, const int* joints, const double* spds) override;
#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTargetVelocity(const int joint, double* vel) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTargetVelocities(double* vels) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getTargetVelocities(const int n_joint, const int* joints, double* vels) override;
#else
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefVelocity(const int joint, double* vel) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefVelocities(double* vels) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefVelocities(const int n_joint, const int* joints, double* vels) override;
#endif

    // ICurrentControl

    YARP_DEV_RETURN_VALUE_TYPE_CH40 getNumberOfMotors(int* ax) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getCurrent(int m, double* curr) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getCurrents(double* currs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getCurrentRange(int m, double* min, double* max) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getCurrentRanges(double* min, double* max) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefCurrents(const double* currs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefCurrent(int m, double curr) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRefCurrents(const int n_motor, const int* motors, const double* currs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefCurrents(double* currs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRefCurrent(int m, double* curr) override;

    // IPidControl

    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPid(const PidControlTypeEnum& pidtype, int j, const Pid& pid) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPids(const PidControlTypeEnum& pidtype, const Pid* pids) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPidReference(const PidControlTypeEnum& pidtype, int j, double ref) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPidReferences(const PidControlTypeEnum& pidtype, const double* refs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPidErrorLimits(const PidControlTypeEnum& pidtype, const double* limits) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidError(const PidControlTypeEnum& pidtype, int j, double* err) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidErrors(const PidControlTypeEnum& pidtype, double* errs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidOutput(const PidControlTypeEnum& pidtype, int j, double* out) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidOutputs(const PidControlTypeEnum& pidtype, double* outs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPid(const PidControlTypeEnum& pidtype, int j, Pid* pid) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPids(const PidControlTypeEnum& pidtype, Pid* pids) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidReference(const PidControlTypeEnum& pidtype, int j, double* ref) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidReferences(const PidControlTypeEnum& pidtype, double* refs) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double* limit) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidErrorLimits(const PidControlTypeEnum& pidtype, double* limits) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 resetPid(const PidControlTypeEnum& pidtype, int j) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 disablePid(const PidControlTypeEnum& pidtype, int j) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 enablePid(const PidControlTypeEnum& pidtype, int j) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPidOffset(const PidControlTypeEnum& pidtype, int j, double v) override;
#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setPidFeedforward(const PidControlTypeEnum& pidtype, int j, double v) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidOffset(const PidControlTypeEnum& pidtype, int j, double& v) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidFeedforward(const PidControlTypeEnum& pidtype, int j, double& v) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidExtraInfo(const PidControlTypeEnum& pidtype, int j, yarp::dev::PidExtraInfo& info) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getPidExtraInfos(const PidControlTypeEnum& pidtype, std::vector<yarp::dev::PidExtraInfo>& info) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool& enabled) override;
#else
    YARP_DEV_RETURN_VALUE_TYPE_CH40 isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) override;
#endif

    // IControlBoardData

    void setControlBoardData(::gzyarp::ControlBoardData* controlBoardData) override;

private:
    std::string m_controlBoardId;
    ::gzyarp::ControlBoardData* m_controlBoardData;

    bool checkRefTorqueIsValid(double refTorque);
};

} // namespace gzyarp
} // namespace dev
} // namespace yarp
