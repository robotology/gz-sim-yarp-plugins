#include <ControlBoard.hh>

#include <Common.hh>
#include <ConfigurationHelpers.hh>
#include <ControlBoardData.hh>
#include <ControlBoardDriver.hh>
#include <ControlBoardTrajectory.hh>
#include <DeviceRegistry.hh>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/msgs/details/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <sdf/Element.hh>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/PidEnums.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

using namespace gz;
using namespace sim;
using namespace systems;
using yarp::os::Bottle;

namespace gzyarp
{
using gz::msgs::Wrench;

ControlBoard::ControlBoard()
    : m_deviceRegistered(false)
{
}

ControlBoard::~ControlBoard()
{
    if (m_deviceRegistered)
    {
        DeviceRegistry::getHandler()->removeDevice(*m_ecm, m_deviceId);
        m_deviceRegistered = false;
    }

    if (m_controlBoardDriver.isValid())
    {
        m_controlBoardDriver.close();
    }
}

void ControlBoard::Configure(const Entity& _entity,
                             const std::shared_ptr<const sdf::Element>& _sdf,
                             EntityComponentManager& _ecm,
                             EventManager& _eventMgr)
{
    using ControlBoardDriverCreator
        = ::yarp::dev::DriverCreatorOf<yarp::dev::gzyarp::ControlBoardDriver>;
    yarp::dev::Drivers::factory().add(
        new ControlBoardDriverCreator("gazebo_controlboard", "", "ControlBoardDriver"));

    bool wipe = false;

    m_ecm = &_ecm;

    if (ConfigurationHelpers::loadPluginConfiguration(_sdf, m_pluginParameters))
    {
        if (!m_pluginParameters.check("yarpDeviceName"))
        {
            yError() << "gz-sim-yarp-controlboard-system : missing yarpDeviceName parameter";
            return;
        }

        yInfo() << "gz-sim-yarp-controlboard-system: configuration of device "
                << m_pluginParameters.find("yarpDeviceName").asString() << " loaded";
    } else
    {
        yError() << "gz-sim-yarp-controlboard-system : missing configuration";
        return;
    }

    std::string yarpDeviceName = m_pluginParameters.find("yarpDeviceName").asString();

    m_robotScopedName = gz::sim::scopedName(_entity, _ecm, "/");
    yDebug() << "gz-sim-yarp-controlboard-system : robot scoped name: " << m_robotScopedName;
    yDebug() << "gz-sim-yarp-controlboard-system : yarpDeviceName: " << yarpDeviceName;

    m_modelEntity = _entity;

    m_pluginParameters.put(yarp::dev::gzyarp::YarpControlBoardScopedName.c_str(),
                           m_robotScopedName.c_str());

    m_pluginParameters.put("device", "gazebo_controlboard");

    if (_sdf->HasElement("initialConfiguration"))
    {
        std::string initialConfiguration = _sdf->Get<std::string>("initialConfiguration");
        yDebug() << "gz-sim-yarp-controlboard-system: initialConfiguration: "
                 << initialConfiguration;
        m_pluginParameters.put("initialConfiguration", initialConfiguration);
    }

    if (!m_controlBoardDriver.open(m_pluginParameters))
    {
        yError() << "gz-sim-yarp-controlboard-system Plugin failed: error in opening yarp "
                    "driver";
        return;
    }

    if (!DeviceRegistry::getHandler()
             ->setDevice(_entity, _ecm, yarpDeviceName, &m_controlBoardDriver, m_deviceId))
    {
        yError() << "gz-sim-yarp-basestate-system: failed setting scopedDeviceName(="
                 << m_robotScopedName << ")";
        return;
    }

    IControlBoardData* iControlBoardData = nullptr;
    auto viewOk = m_controlBoardDriver.view(iControlBoardData);

    if (!viewOk || !iControlBoardData)
    {
        yError() << "gz-sim-yarp-controlboard-system Plugin failed: error in getting "
                    "IControlBoardData interface";
        return;
    }
    iControlBoardData->setControlBoardData(&m_controlBoardData);

    if (!setJointProperties(_ecm))
    {
        yError() << "gz-sim-yarp-controlboard-system: failed setting joint properties";
        return;
    }

    resetPositionsAndTrajectoryGenerators(_ecm);

    yInfo() << "Registered YARP device with instance name:" << m_deviceId;
    m_deviceRegistered = true;
}

void ControlBoard::PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    if (!updateTrajectories(_info, _ecm))
    {
        yError() << "Error while updating trajectories";
    }

    if (!updateReferences(_info, _ecm))
    {
        yError() << "Error while updating control references";
    }
}

void ControlBoard::PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm)
{
    updateSimTime(_info);

    if (!readJointsMeasurements(_ecm))
    {
        yError() << "Error while reading joints measurements";
    }

    // checkForJointsHwFault();
}

void ControlBoard::Reset(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    resetPositionsAndTrajectoryGenerators(_ecm);
}

// Private methods

bool ControlBoard::setJointProperties(EntityComponentManager& _ecm)
{
    std::vector<std::string> jointNames{};
    bool paramOk = gzyarp::readVectorFromConfigFile(m_pluginParameters, "jointNames", jointNames);

    if (!paramOk)
    {
        yError() << "Error while reading jointNames parameter from plugin parameters";
        return false;
    }

    auto jointsFromConfigNum = jointNames.size();
    yInfo() << "Found " + std::to_string(jointsFromConfigNum)
                   + " joints from the plugin configuration.";

    {
        std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

        m_controlBoardData.joints.resize(jointsFromConfigNum);

        auto model = Model(m_modelEntity);
        auto jointEntititesCount = model.JointCount(_ecm);
        yInfo() << "Found " + std::to_string(jointEntititesCount)
                       + " joints from the model description.";

        m_controlBoardData.joints.resize(jointsFromConfigNum);
        for (size_t i = 0; i < jointsFromConfigNum; i++)
        {
            std::string jointFromConfigName = jointNames.at(i);

            auto jointEntity = model.JointByName(_ecm, jointFromConfigName);
            if (!jointEntity)
            {
                yError() << "Joint " << jointFromConfigName
                         << " not found in the model description.";
                return false;
            }

            // Enable position, velocity and wrench check
            auto gzJoint = Joint(jointEntity);
            gzJoint.EnablePositionCheck(_ecm, true);
            gzJoint.EnableVelocityCheck(_ecm, true);
            gzJoint.EnableTransmittedWrenchCheck(_ecm, true);

            // Initialize JointProperties object
            m_controlBoardData.joints[i].name = jointFromConfigName;

            yInfo() << "Joint " << jointFromConfigName << " added to the control board data.";
        }

        if (!initializeJointPositionLimits(_ecm))
        {
            yError() << "Error while setting joint position limits";
            return false;
        }

        if (!initializeTrajectoryGenerators())
        {
            yError() << "Error while setting trajectory generators";
            return false;
        }

        if (!initializePIDsForPositionControl())
        {
            yError() << "Error while initializing PIDs";
            return false;
        }

    } // lock_guard

    return true;
}

void ControlBoard::updateSimTime(const UpdateInfo& _info)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    m_controlBoardData.simTime.update(_info.simTime.count() / 1e9);
}

bool ControlBoard::readJointsMeasurements(const gz::sim::EntityComponentManager& _ecm)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    auto model = Model(m_modelEntity);
    Joint gzJoint;
    for (auto& joint : m_controlBoardData.joints)
    {
        try
        {
            gzJoint = Joint(model.JointByName(_ecm, joint.name));
        } catch (const std::exception& e)
        {
            yError() << "Error while trying to access joint " << joint.name;
            return false;
        }

        if (gzJoint.Position(_ecm).has_value())
        {
            joint.position = convertGazeboToUser(joint, gzJoint.Position(_ecm).value().at(0));
        } else
        {
            yError() << "Error while reading position for joint " << joint.name;
            return false;
        }

        if (gzJoint.Velocity(_ecm).has_value())
        {
            joint.velocity = convertGazeboToUser(joint, gzJoint.Velocity(_ecm).value().at(0));
        } else
        {
            yError() << "Error while reading velocity for joint " << joint.name;
            return false;
        }

        if (gzJoint.TransmittedWrench(_ecm).has_value())
        {
            // joint.torque = gzJoint.TransmittedWrench(_ecm).value().at(0).torque().x();
            joint.torque
                = getJointTorqueFromTransmittedWrench(gzJoint,
                                                      gzJoint.TransmittedWrench(_ecm).value().at(0),
                                                      _ecm);
        } else
        {
            yError() << "Error while reading torque for joint " << joint.name;
            return false;
        }
    }
    return true;
}

double
ControlBoard::getJointTorqueFromTransmittedWrench(const Joint& gzJoint,
                                                  const Wrench& wrench,
                                                  const gz::sim::EntityComponentManager& ecm) const
{

    auto axis = gzJoint.Axis(ecm).value().at(0).Xyz();
    // TODO manage different types of joints

    // Revolute Joint
    // Motion subspace s = [0_3; axis]
    auto wrench_torque
        = gz::math::Vector3d(wrench.torque().x(), wrench.torque().y(), wrench.torque().z());
    double torque = wrench_torque.Dot(axis);
    return torque;
}

void ControlBoard::checkForJointsHwFault()
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    for (auto& joint : m_controlBoardData.joints)
    {
        if (joint.controlMode != VOCAB_CM_HW_FAULT && std::abs(joint.torque) > joint.maxTorqueAbs)
        {
            joint.controlMode = VOCAB_CM_HW_FAULT;
            yError() << "An hardware fault occurred on joint " << joint.name
                     << " torque too big! ( " << joint.torque << " )";
        }
    }
}

bool ControlBoard::updateTrajectories(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    // TODO: execute the following at control update time

    for (auto& joint : m_controlBoardData.joints)
    {
        switch (joint.controlMode)
        {
        case VOCAB_CM_POSITION:
            joint.refPosition = joint.trajectoryGenerator->computeTrajectory();
            joint.isMotionDone = joint.trajectoryGenerator->isMotionDone();
            break;
        case VOCAB_CM_MIXED:
            // TODO when implementing mixed control mode
            yError() << "Control mode MIXED not implemented yet";
            break;
        case VOCAB_CM_VELOCITY:
            // TODO when implementing velocity control mode
            yError() << "Control mode VELOCITY not implemented yet";
            break;
        }
    }

    return true;
}

bool ControlBoard::updateReferences(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    double forceReference{};
    Joint gzJoint;

    for (auto& joint : m_controlBoardData.joints)
    {
        switch (joint.controlMode)
        {
        case VOCAB_CM_TORQUE:
            forceReference = joint.refTorque;
            break;
        case VOCAB_CM_POSITION:
        case VOCAB_CM_POSITION_DIRECT: {
            // TODO manage motor positions instead of joint positions when implemented
            auto& pid = joint.pidControllers[yarp::dev::VOCAB_PIDTYPE_POSITION];
            forceReference = pid.Update(convertUserToGazebo(joint, joint.position)
                                            - convertUserToGazebo(joint, joint.refPosition),
                                        _info.dt);
            break;
        }
        case VOCAB_CM_IDLE:
        case VOCAB_CM_HW_FAULT:
            forceReference = 0.0;
            break;
        default:
            yWarning() << "Joint " << joint.name << " control mode " << joint.controlMode
                       << " is currently not implemented";
            return false;
        };

        // TODO check if joint is within limits

        try
        {
            gzJoint = Joint(Model(m_modelEntity).JointByName(_ecm, joint.name));
        } catch (const std::exception& e)
        {
            yError() << "Error while trying to access joint " << joint.name;
            return false;
        }

        std::vector<double> forceVec{forceReference};

        gzJoint.SetForce(_ecm, forceVec);
    }

    return true;
}

bool ControlBoard::initializePIDsForPositionControl()
{
    if (!m_pluginParameters.check("POSITION_CONTROL"))
    {
        yError() << "Group POSITION_CONTROL not found in plugin configuration";
        return false;
    }
    Bottle pidGroup = m_pluginParameters.findGroup("POSITION_CONTROL");

    size_t numberOfJoints = m_controlBoardData.joints.size();
    auto cUnits = AngleUnitEnum::DEG;

    // control units block
    yarp::os::Value controlUnitsValue = pidGroup.find("controlUnits");
    if (!controlUnitsValue.isNull() && controlUnitsValue.isString())
    {
        if (controlUnitsValue.asString() == "metric_units")
        {
            cUnits = AngleUnitEnum::DEG;
        } else if (controlUnitsValue.asString() == "si_units")
        {
            cUnits = AngleUnitEnum::RAD;
        } else
        {
            yError() << "invalid controlUnits value";
            return false;
        }
    } else
    {
        yError() << "POSITION_CONTROL: 'controlUnits' param missing. Cannot "
                    "continue";
        return false;
    }

    // control law block
    yarp::os::Value controlLawValue = pidGroup.find("controlLaw");
    if (!controlLawValue.isNull() && controlLawValue.isString())
    {
        if (controlLawValue.asString() == "joint_pid_gazebo_v1")
        {
            for (size_t i = 0; i < numberOfJoints; i++)
                m_controlBoardData.joints[i].positionControlLaw = "joint_pid_gazebo_v1";
        } else
        {
            yError() << "invalid controlLaw value";
            return false;
        }
    } else
    {
        yError() << "POSITION_CONTROL: 'controlLaw' parameter missing";
        return false;
    }

    std::vector<yarp::dev::Pid> yarpPIDs(numberOfJoints);

    std::vector<std::pair<std::string, std::string>> parameters
        = {{"kp", "Pid kp parameter"},
           {"kd", "Pid kd parameter"},
           {"ki", "Pid ki parameter"},
           {"maxInt", "Pid maxInt parameter"},
           {"maxOutput", "Pid maxOutput parameter"},
           {"shift", "Pid shift parameter"},
           {"ko", "Pid ko parameter"},
           {"stictionUp", "Pid stictionUp"},
           {"stictionDwn", "Pid stictionDwn"}};

    for (const auto& param : parameters)
    {
        std::vector<double> pidParams{};
        if (!tryGetGroup(pidGroup, pidParams, param.first, param.second, numberOfJoints))
        {
            return false;
        }
        setYarpPIDsParam(pidParams, param.first, yarpPIDs, numberOfJoints);
    }

    setJointPositionPIDs(cUnits, yarpPIDs);

    return true;
}

bool ControlBoard::tryGetGroup(const Bottle& in,
                               std::vector<double>& out,
                               const std::string& key,
                               const std::string& txt,
                               int expectedSize)
{
    bool vecOk = gzyarp::readVectorFromConfigFile(in, key, out);
    if (!vecOk)
    {
        yError() << key << " not found";
        return false;
    }
    if (out.size() != expectedSize)
    {
        yError() << "Incorrect number of entries for group: " << key;
        return false;
    }

    return true;
}

bool ControlBoard::setYarpPIDsParam(const std::vector<double>& pidParams,
                                    const std::string& paramName,
                                    std::vector<yarp::dev::Pid>& yarpPIDs,
                                    size_t numberOfJoints)
{

    std::unordered_map<std::string, int> pidParamNameMap = {{"kp", 0},
                                                            {"kd", 1},
                                                            {"ki", 2},
                                                            {"maxInt", 3},
                                                            {"maxOutput", 4},
                                                            {"shift", 5},
                                                            {"ko", 6},
                                                            {"stictionUp", 7},
                                                            {"stictionDwn", 8}};

    for (size_t i = 0; i < numberOfJoints; i++)
    {
        switch (pidParamNameMap[paramName])
        {
        case 0:
            yarpPIDs[i].kp = pidParams.at(i);
            break;
        case 1:
            yarpPIDs[i].kd = pidParams.at(i);
            break;
        case 2:
            yarpPIDs[i].ki = pidParams.at(i);
            break;
        case 3:
            yarpPIDs[i].max_int = pidParams.at(i);
            break;
        case 4:
            yarpPIDs[i].max_output = pidParams.at(i);
            break;
        case 5:
            yarpPIDs[i].scale = pidParams.at(i);
            break;
        case 6:
            yarpPIDs[i].offset = pidParams.at(i);
            break;
        case 7:
            yarpPIDs[i].stiction_up_val = pidParams.at(i);
            break;
        case 8:
            yarpPIDs[i].stiction_down_val = pidParams.at(i);
            break;
        default:
            yError() << "Invalid parameter name";
            return false;
        }
    }

    return true;
}

void ControlBoard::setJointPositionPIDs(AngleUnitEnum cUnits,
                                        const std::vector<yarp::dev::Pid>& yarpPIDs)
{
    for (size_t i = 0; i < m_controlBoardData.joints.size(); i++)
    {
        auto& jointPositionPID
            = m_controlBoardData.joints[i].pidControllers[yarp::dev::VOCAB_PIDTYPE_POSITION];

        if (cUnits == AngleUnitEnum::DEG)
        {
            auto& joint = m_controlBoardData.joints.at(i);
            jointPositionPID.SetPGain(convertUserGainToGazeboGain(joint, yarpPIDs[i].kp)
                                      / pow(2, yarpPIDs[i].scale));
            jointPositionPID.SetIGain(convertUserGainToGazeboGain(joint, yarpPIDs[i].ki)
                                      / pow(2, yarpPIDs[i].scale));
            jointPositionPID.SetDGain(convertUserGainToGazeboGain(joint, yarpPIDs[i].kd)
                                      / pow(2, yarpPIDs[i].scale));
        } else if (cUnits == AngleUnitEnum::RAD)
        {
            jointPositionPID.SetPGain(yarpPIDs[i].kp / pow(2, yarpPIDs[i].scale));
            jointPositionPID.SetIGain(yarpPIDs[i].ki / pow(2, yarpPIDs[i].scale));
            jointPositionPID.SetDGain(yarpPIDs[i].kd / pow(2, yarpPIDs[i].scale));
        }

        jointPositionPID.SetIMax(yarpPIDs[i].max_int);
        jointPositionPID.SetIMin(-yarpPIDs[i].max_int);
        jointPositionPID.SetCmdMax(yarpPIDs[i].max_output);
        jointPositionPID.SetCmdMin(-yarpPIDs[i].max_output);
    }
}

double ControlBoard::convertUserGainToGazeboGain(JointProperties& joint, double value)
{
    // TODO discriminate between joint types
    return gzyarp::convertDegreeGainToRadianGains(value);
}

double ControlBoard::convertGazeboGainToUserGain(JointProperties& joint, double value)
{
    // TODO discriminate between joint types
    return gzyarp::convertRadianGainToDegreeGains(value);
}

double ControlBoard::convertGazeboToUser(JointProperties& joint, double value)
{
    // TODO discriminate between joint types
    return convertRadiansToDegrees(value);
}

double ControlBoard::convertUserToGazebo(JointProperties& joint, double value)
{
    // TODO discriminate between joint types
    return convertDegreesToRadians(value);
}

bool ControlBoard::initializeJointPositionLimits(const gz::sim::EntityComponentManager& ecm)
{
    if (!m_pluginParameters.check("LIMITS"))
    {
        yError() << "Group LIMITS not found in plugin configuration";
        return false;
    }

    Bottle limitsGroup = m_pluginParameters.findGroup("LIMITS");
    size_t numberOfJoints = m_controlBoardData.joints.size();
    std::vector<double> limitMinGroup, limitMaxGroup;

    if (!(tryGetGroup(limitsGroup, limitMinGroup, "jntPosMin", "", numberOfJoints)
          && tryGetGroup(limitsGroup, limitMaxGroup, "jntPosMax", "", numberOfJoints)))
    {
        yError() << "Error while reading joint position limits from plugin configuration";
        return false;
    }

    for (size_t i = 0; i < numberOfJoints; ++i)
    {
        // TODO: access gazebo joint position limits and use them to check if software limits
        // ([LIMITS] group) are consistent. In case they are not defined set them as sw limits.
        auto& joint = m_controlBoardData.joints[i];
        joint.positionLimitMin = limitMinGroup.at(i);
        joint.positionLimitMax = limitMaxGroup.at(i);
    }

    return true;
}

bool ControlBoard::initializeTrajectoryGenerators()
{
    // Read from configuration
    auto trajectoryGeneratorsGroup = m_pluginParameters.findGroup("TRAJECTORY_GENERATION");
    bool missingConfiguration = false;
    if (trajectoryGeneratorsGroup.isNull())
    {
        yWarning() << "Group TRAJECTORY_GENERATION not found in plugin configuration. Defaults to "
                      "minimum jerk trajectory.";
        missingConfiguration = true;
    }

    auto trajectoryTypeValue = trajectoryGeneratorsGroup.find("trajectory_type");
    if (!missingConfiguration && (trajectoryTypeValue.isNull() || !trajectoryTypeValue.isString()))
    {
        yWarning() << "Group trajectoryType not found in TRAJECTORY_GENERATION group. Defaults to "
                      "minimum jerk trajectory";
        missingConfiguration = true;
    }

    yarp::dev::gzyarp::TrajectoryType trajectoryType;

    if (missingConfiguration)
    {
        trajectoryType = yarp::dev::gzyarp::TrajectoryType::TRAJECTORY_TYPE_MIN_JERK;

    } else
    {
        std::unordered_map<std::string, yarp::dev::gzyarp::TrajectoryType> trajectoryTypeMap
            = {{"constant_speed", yarp::dev::gzyarp::TrajectoryType::TRAJECTORY_TYPE_CONST_SPEED},
               {"minimum_jerk", yarp::dev::gzyarp::TrajectoryType::TRAJECTORY_TYPE_MIN_JERK},
               {"trapezoidal_speed",
                yarp::dev::gzyarp::TrajectoryType::TRAJECTORY_TYPE_TRAP_SPEED}};

        try
        {
            trajectoryType = trajectoryTypeMap.at(trajectoryTypeValue.asString());
            yInfo() << "Trajectory generator type set to " << trajectoryTypeValue.asString();
        } catch (const std::out_of_range& e)
        {
            yError() << "Invalid trajectory type " << trajectoryTypeValue.asString()
                     << " specified in trajectoryType parameter. Defaults to minimum jerk "
                        "trajectory";
        }
    }

    for (auto& joint : m_controlBoardData.joints)
    {
        joint.trajectoryGenerator
            = yarp::dev::gzyarp::TrajectoryGeneratorFactory::create(trajectoryType);
    }

    if (!initializeTrajectoryGeneratorReferences(trajectoryGeneratorsGroup))
    {
        yError() << "Error while initializing trajectory generator references";
        return false;
    }

    return true;
}

bool ControlBoard::initializeTrajectoryGeneratorReferences(Bottle& trajectoryGeneratorsGroup)
{
    bool useDefaultSpeedRef{false}, useDefaultAccelerationRef{false};
    std::vector<double> refSpeedGroup, refAccelerationGroup;

    if (!trajectoryGeneratorsGroup.isNull())
    {
        // Read from configuration
        if (!tryGetGroup(trajectoryGeneratorsGroup,
                         refSpeedGroup,
                         "refSpeed",
                         "",
                         m_controlBoardData.joints.size()))
        {
            yWarning() << "Parameter refSpeed not found in TRAJECTORY_GENERATION group. Defaults "
                          "will be applied";
            useDefaultSpeedRef = true;
        }

        if (!tryGetGroup(trajectoryGeneratorsGroup,
                         refAccelerationGroup,
                         "refAcceleration",
                         "",
                         m_controlBoardData.joints.size()))
        {
            yWarning() << "Parameter refAcceleration not found in TRAJECTORY_GENERATION group. "
                          "Defaults will be applied";
            useDefaultAccelerationRef = true;
        }
    } else
    {
        // Use defaults
        yWarning() << "Group TRAJECTORY_GENERATION not found in plugin configuration. Defaults "
                      "trajectory generation reference speed and accelerations will be applied";
        useDefaultSpeedRef = true;
        useDefaultAccelerationRef = true;
    }

    // Set trajectory generation reference speed and acceleration
    // TODO: manage different joint types
    for (size_t i = 0; i < m_controlBoardData.joints.size(); ++i)
    {
        auto& joint = m_controlBoardData.joints[i];
        if (useDefaultSpeedRef)
        {
            joint.trajectoryGenerationRefSpeed = 10.0; // [deg/s]
        } else
        {
            joint.trajectoryGenerationRefSpeed = refSpeedGroup.at(i);
        }

        if (useDefaultAccelerationRef)
        {
            joint.trajectoryGenerationRefAcceleration = 10.0; // [deg/s^2]
        } else
        {
            joint.trajectoryGenerationRefAcceleration = refAccelerationGroup.at(i);
        }

        // Clip trajectory generation reference velocities according to max joint limit
        if (joint.trajectoryGenerationRefSpeed > joint.velocityLimitMax)
        {
            joint.trajectoryGenerationRefSpeed = joint.velocityLimitMax;
        }

        yDebug() << "Joint " << joint.name
                 << " trajectory generation reference speed: " << joint.trajectoryGenerationRefSpeed
                 << " [deg/s]";
        yDebug() << "Joint " << joint.name << " trajectory generation reference acceleration: "
                 << joint.trajectoryGenerationRefAcceleration << " [deg/s^2]";
    }

    return true;
}

bool ControlBoard::parseInitialConfiguration(std::vector<double>& initialConfigurations)
{

    if (!m_pluginParameters.check("initialConfiguration"))
    {
        return false;
    }

    std::stringstream ss(m_pluginParameters.find("initialConfiguration").toString());

    double tmp{};
    size_t counter = 0;

    while (ss >> tmp)
    {
        if (counter >= m_controlBoardData.joints.size())
        {
            yWarning() << "Too many elements in initial configuration, stopping at element "
                       << (counter + 1);
            break;
        }

        initialConfigurations[counter++] = tmp;
    }

    return true;
}

void ControlBoard::resetPositionsAndTrajectoryGenerators(gz::sim::EntityComponentManager& ecm)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    std::vector<double> initialConfigurations(m_controlBoardData.joints.size());
    if (parseInitialConfiguration(initialConfigurations))
    {
        yInfo() << "Initial configuration found, initializing trajectory generator with it";

        for (size_t i = 0; i < m_controlBoardData.joints.size(); i++)
        {
            auto& joint = m_controlBoardData.joints.at(i);
            auto gzPos = initialConfigurations[i];
            auto userPos = convertGazeboToUser(joint, gzPos);
            // Reset joint properties
            joint.trajectoryGenerationRefPosition = userPos;
            joint.refPosition = userPos;
            joint.position = userPos;
            // Reset position of gazebo joint
            // TODO(xela-95): store joint entity in JointProperties
            Joint(Model(m_modelEntity).JointByName(ecm, joint.name))
                .ResetPosition(ecm, std::vector<double>{gzPos});
            auto limitMin = joint.positionLimitMin;
            auto limitMax = joint.positionLimitMax;
            joint.trajectoryGenerator->setLimits(limitMin, limitMax);
            joint.trajectoryGenerator->initTrajectory(joint.position,
                                                      joint.position,
                                                      joint.trajectoryGenerationRefSpeed,
                                                      joint.trajectoryGenerationRefAcceleration,
                                                      m_controlBoardData.controlUpdatePeriod);
        }

    } else
    {
        yWarning() << "No initial configuration found, initializing trajectory generator with "
                      "current values";

        for (size_t i = 0; i < m_controlBoardData.joints.size(); i++)
        {
            auto& joint = m_controlBoardData.joints.at(i);
            auto gzJoint = Joint(Model(m_modelEntity).JointByName(ecm, joint.name));
            if (gzJoint.Position(ecm).has_value() && gzJoint.Position(ecm).value().size() > 0)
            {
                auto gzPos = gzJoint.Position(ecm).value().at(0);
                auto userPos = convertGazeboToUser(joint, gzPos);
                // Reset joint properties
                joint.trajectoryGenerationRefPosition = userPos;
                joint.refPosition = userPos;
                joint.position = userPos;
            }
            auto limitMin = joint.positionLimitMin;
            auto limitMax = joint.positionLimitMax;
            joint.trajectoryGenerator->setLimits(limitMin, limitMax);
            joint.trajectoryGenerator->initTrajectory(joint.position,
                                                      joint.position,
                                                      joint.trajectoryGenerationRefSpeed,
                                                      joint.trajectoryGenerationRefAcceleration,
                                                      m_controlBoardData.controlUpdatePeriod);
        }
    }

    // Reset control mode
    for (auto& joint : m_controlBoardData.joints)
    {
        joint.controlMode = VOCAB_CM_POSITION;
    }
}

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::ControlBoard,
              gz::sim::System,
              gzyarp::ControlBoard::ISystemConfigure,
              gzyarp::ControlBoard::ISystemPreUpdate,
              gzyarp::ControlBoard::ISystemPostUpdate,
              gzyarp::ControlBoard::ISystemReset)
