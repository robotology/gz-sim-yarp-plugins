#pragma once

#include <ControlBoardData.hh>

#include <cstddef>
#include <gz/msgs/details/wrench.pb.h>
#include <memory>
#include <string>

#include <gz/msgs/joint_wrench.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <sdf/Element.hh>

#include <vector>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

namespace gzyarp
{

class ControlBoard : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate,
                     public gz::sim::ISystemReset
{

public:
    ControlBoard();
    ~ControlBoard();

    // ISystemConfigure interface
    void Configure(const gz::sim::Entity& _entity,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   gz::sim::EntityComponentManager& _ecm,
                   gz::sim::EventManager& _eventMgr) override;

    // ISystemPreUpdate interface
    void
    PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;

    // ISystemPostUpdate interface
    void PostUpdate(const gz::sim::UpdateInfo& _info,
                    const gz::sim::EntityComponentManager& _ecm) override;

    // ISystemReset interface
    void Reset(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;

private:
    bool m_deviceRegistered;
    std::string m_robotScopedName;
    std::string m_deviceId;
    gz::sim::Entity m_modelEntity;
    yarp::dev::PolyDriver m_controlBoardDriver;
    ControlBoardData m_controlBoardData;
    yarp::os::Network m_yarpNetwork;
    yarp::os::Property m_pluginParameters;
    gz::sim::EntityComponentManager* m_ecm;

    enum class AngleUnitEnum
    {
        DEG = 0,
        RAD = 1
    };

    bool setJointProperties(gz::sim::EntityComponentManager& _ecm);
    void updateSimTime(const gz::sim::UpdateInfo& _info);
    bool readJointsMeasurements(const gz::sim::EntityComponentManager& _ecm);
    void checkForJointsHwFault();
    bool
    updateTrajectories(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm);
    bool updateReferences(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm);
    double getJointTorqueFromTransmittedWrench(const gz::sim::Joint& gzJoint,
                                               const gz::msgs::Wrench& wrench,
                                               const gz::sim::EntityComponentManager& ecm) const;
    bool initializePIDsForPositionControl();
    bool tryGetGroup(const yarp::os::Bottle& in,
                     std::vector<double>& out,
                     const std::string& key,
                     const std::string& txt,
                     int expectedSize);
    bool setYarpPIDsParam(const std::vector<double>& pidParams,
                          const std::string& paramName,
                          std::vector<yarp::dev::Pid>& yarpPIDs,
                          size_t numberOfJoints);
    void setJointPositionPIDs(AngleUnitEnum cUnits, const std::vector<yarp::dev::Pid>& yarpPIDs);
    double convertUserGainToGazeboGain(JointProperties& joint, double value);
    double convertGazeboGainToUserGain(JointProperties& joint, double value);
    double convertGazeboToUser(JointProperties& joint, double value);
    double convertUserToGazebo(JointProperties& joint, double value);
    bool initializeJointPositionLimits(const gz::sim::EntityComponentManager& ecm);
    bool initializeTrajectoryGenerators();
    bool initializeTrajectoryGeneratorReferences(yarp::os::Bottle& trajectoryGeneratorsGroup);
    bool parseInitialConfiguration(std::vector<double>& initialConfigurations);
    void resetPositionsAndTrajectoryGenerators(gz::sim::EntityComponentManager& ecm);
};

} // namespace gzyarp
