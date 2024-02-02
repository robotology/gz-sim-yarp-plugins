#pragma once

#include "ControlBoardData.hh"
#include "ControlBoardDataSingleton.hh"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/ITorqueControl.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{

const std::string YarpControlBoardScopedName = "robotScopedName";

class ControlBoardDriver : public DeviceDriver, public IInteractionMode
//    public IControlMode,
//    public ITorqueControl,
//    public IAxisInfo,
//    public IEncodersTimed
{
public:
    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IInteractionMode
    bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode) override;
    bool
    getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes) override;
    bool getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
    bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    bool
    setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes) override;
    bool setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

    // // IControlMode
    // bool getControlMode(int j, int* mode) override;
    // bool getControlModes(int* modes) override;
    // bool getControlModes(const int n_joint, const int* joints, int* modes) override;
    // bool setControlMode(const int j, const int mode) override;
    // bool setControlModes(const int n_joint, const int* joints, int* modes) override;
    // bool setControlModes(int* modes) override;

    // // ITorqueControl
    // bool getAxes(int* ax) override;
    // bool getRefTorques(double* t) override;
    // bool getRefTorque(int j, double* t) override;
    // bool setRefTorques(const double* t) override;
    // bool setRefTorque(int j, double t) override;
    // bool setRefTorques(const int n_joint, const int* joints, const double* t) override;
    // bool getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params) override;
    // bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;
    // bool getTorque(int j, double* t) override;
    // bool getTorques(double* t) override;
    // bool getTorqueRange(int j, double* min, double* max) override;
    // bool getTorqueRanges(double* min, double* max) override;

    // // IAxisInfo
    // bool getAxisName(int axis, std::string& name) override;
    // bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

    // // IEncodersTimed
    // bool getEncodersTimed(double* encs, double* time) override;
    // bool getEncoderTimed(int j, double* encs, double* time) override;

private:
    std::string m_controlBoardScopedName;

    ControlBoardData* m_controlBoardData;
};

} // namespace gzyarp
} // namespace dev
} // namespace yarp
