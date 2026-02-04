#pragma once

#include <ControlBoardTrajectory.hh>

#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/conf/numeric.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IJointCoupling.h>
#include <yarp/dev/PidEnums.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Vocab.h>

#include <gz/math/PID.hh>

namespace gzyarp
{

struct PidControlTypeEnumHashFunction
{
    size_t operator()(const yarp::dev::PidControlTypeEnum& key) const
    {
        std::size_t hash = std::hash<int>()(static_cast<int>(key));
        return hash;
    }
};

struct CommonJointProperties {
    std::string name;
    yarp::dev::InteractionModeEnum interactionMode{yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF};
    yarp::conf::vocab32_t controlMode{VOCAB_CM_IDLE};
    // TODO(xela95): Update unit of measurements once we support prismatic joints
    double refTorque{0.0}; // Desired reference torques for torque control mode [Nm]
    double torque{0.0}; // Measured torques [Nm]
    double maxTorqueAbs{0.0}; // Maximum torque absolute value [Nm]
    double zeroPosition{0.0}; // The zero position is the position of the GAZEBO joint that will be
                              // read as the starting one i.e.
                              // getEncoder(j)=m_zeroPosition+gazebo.getEncoder(j);
    double refPosition{0.0};  //for position control mode
    double refVelocity{0.0};  //for velocity control mode
    double position{0.0}; // Joint position [deg]
    double positionLimitMin{std::numeric_limits<double>::min()};
    double positionLimitMax{std::numeric_limits<double>::max()};
    double velocity{0.0}; // Joint velocity [deg/s]
    double velocityLimitMin{std::numeric_limits<double>::min()};
    double velocityLimitMax{std::numeric_limits<double>::max()};
    double acceleration{0.0}; // Joint acceleration [deg/s^2]
};

struct PhysicalJointProperties
{
    CommonJointProperties commonJointProperties;
    std::unordered_map<yarp::dev::PidControlTypeEnum, gz::math::PID, PidControlTypeEnumHashFunction>
        pidControllers;
    std::string positionControlLaw; // TODO: verify usefulness of this field
#if (YARP_VERSION_MAJOR > 3)
    yarp::dev::PidControlTypeEnum selected_vel_dir_pid = yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_DIRECT_1;
    yarp::dev::PidControlTypeEnum selected_vel_pid = yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_1;
    yarp::dev::PidControlTypeEnum selected_pos_dir_pid = yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_DIRECT_1;
    yarp::dev::PidControlTypeEnum selected_pos_pid = yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_1;
#endif
};

struct ActuatedAxisProperties
{
    CommonJointProperties commonJointProperties;
    std::unique_ptr<yarp::dev::gzyarp::TrajectoryGenerator> trajectoryGenerator;
    std::unique_ptr<yarp::dev::gzyarp::RampFilter> speedRampHandler;
    std::unique_ptr<yarp::dev::gzyarp::Watchdog> velocityWatchdog;
    double trajectoryGenerationRefPosition{0.0};
    double trajectoryGenerationRefSpeed{0.0};
    double trajectoryGenerationRefAcceleration{0.0};
    bool isMotionDone{true};
};

class ControlBoardData
{
public:
    std::mutex mutex;
    std::vector<PhysicalJointProperties> physicalJoints;
    std::vector<ActuatedAxisProperties>  actuatedAxes;
    yarp::os::Stamp simTime;
    yarp::dev::IJointCoupling* ijointcoupling{nullptr};
    // TODO (xela95): read this value from configuration file
    std::chrono::milliseconds controlUpdatePeriod = std::chrono::milliseconds(1);

    bool initCoupledJoints();
    bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);
    bool setControlMode(int j, int mode);

    static double convertUserGainToGazeboGain(PhysicalJointProperties& joint, double value);
    static double convertGazeboGainToUserGain(PhysicalJointProperties& joint, double value);
    static double convertGazeboToUser(PhysicalJointProperties& joint, double value);
    static double convertUserToGazebo(PhysicalJointProperties& joint, double value);
private:
    yarp::sig::VectorOf<size_t> coupledActuatedAxes, coupledPhysicalJoints;
};

class IControlBoardData
{
public:
    virtual void setControlBoardData(ControlBoardData*) = 0;

    virtual ~IControlBoardData() {};
};

} // namespace gzyarp
