#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Sensor.hh>
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/ForceTorque.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/JointTransmittedWrench.hh"
#include "gz/sim/components/Pose.hh"
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <iostream>
#include <sdf/ForceTorque.hh>

using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::Network;

using namespace gz;
using namespace sim;
using namespace systems;
 
// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class ForceTorque
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  // Implement Configure callback, provided by ISystemConfigure
  // and called once at startup.
  virtual void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) override
  {
    this->sensor = Entity(_entity);
    this->joint = Sensor(this->sensor).Parent(_ecm).value();
    std::string childLinkName = Joint(this->joint).ChildLinkName(_ecm).value();
    std::string parentLinkName = Joint(this->joint).ParentLinkName(_ecm).value();
    auto model = Model(Joint(this->joint).ParentModel(_ecm).value());
    this->childLink = model.LinkByName(_ecm, childLinkName);
    this->parentLink = model.LinkByName(_ecm, parentLinkName);
    this->measureFrame = _ecm.Component<components::ForceTorque>(this->sensor)->Data().ForceTorqueSensor()->Frame();
    this->measureDirection = _ecm.Component<components::ForceTorque>(this->sensor)->Data().ForceTorqueSensor()->MeasureDirection();
    this->port.open("/force_torque");
  }
 
  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  virtual void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override
  {
    // From ForceTorquePrivate::Update
    auto jointWrench = _ecm.Component<components::JointTransmittedWrench>(this->joint);
    if (nullptr == jointWrench)
    {
      return;
    }
    // Notation:
    // X_WJ: Pose of joint in world
    // X_WP: Pose of parent link in world
    // X_WC: Pose of child link in world
    // X_WS: Pose of sensor in world
    // X_SP: Pose of parent link in sensors frame
    // X_SC: Pose of child link in sensors frame
    const auto X_WP = worldPose(this->parentLink, _ecm);
    const auto X_WC = worldPose(this->childLink, _ecm);
    const auto X_CJ = _ecm.Component<components::Pose>(this->joint)->Data();
    auto X_WJ = X_WC * X_CJ;
    auto X_JS = _ecm.Component<components::Pose>(this->sensor)->Data();
    auto X_WS = X_WJ * X_JS;
    auto X_SP = X_WS.Inverse() * X_WP;
    auto X_SC = X_WS.Inverse() * X_WC;

    math::Vector3d force =
        X_JS.Rot().Inverse() * msgs::Convert(jointWrench->Data().force());

    math::Vector3d torque =
        X_JS.Rot().Inverse() *
            msgs::Convert(jointWrench->Data().torque()) -
        X_JS.Pos().Cross(force);
      gz::math::Vector3d measuredForce;
      gz::math::Vector3d measuredTorque;

    if (measureFrame == sdf::ForceTorqueFrame::PARENT)
    {
      measuredForce =
          X_SP.Rot().Inverse() * force;
      measuredTorque =
          X_SP.Rot().Inverse() * torque;
    }
    else if (measureFrame == sdf::ForceTorqueFrame::CHILD)
    {
      measuredForce =
          X_SC.Rot().Inverse() * force;
      measuredTorque =
          X_SC.Rot() * torque;
    }
    else if (measureFrame == sdf::ForceTorqueFrame::SENSOR)
    {
      measuredForce = force;
      measuredTorque = torque;
    }
    else
    {
      gzerr << "measureFrame must be PARENT_LINK, CHILD_LINK or SENSOR\n";
    }
  

    if (measureDirection ==
        sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT)
    {
      measuredForce *= -1;
      measuredTorque *= -1;
    }



    Bottle& output = port.prepare();
    std::string msg = std::to_string(_info.simTime.count()/1e9) + 
      "   Force: " + 
      std::to_string(measuredForce.X()) + " " +
      std::to_string(measuredForce.Y()) + " " +
      std::to_string(measuredForce.Z()) + 
      "   Torque: "+
      std::to_string(measuredTorque.X()) + " " +
      std::to_string(measuredTorque.Y()) + " " +
      std::to_string(measuredTorque.Z());
    output.clear();
    output.addString(msg);
    port.write();
    
    // Print force and torque values
    /*
    std::cout << _info.simTime.count()/1e9 << std::endl;
    std::cout << "Force:  " << force << std::endl;
    std::cout << "Torque: " << torque << std::endl << std::endl;
    */
  }
 
  private: 
    Entity sensor;
    Entity joint;
    Entity parentLink;
    Entity childLink;
    BufferedPort<Bottle> port;
    Network yarp;
    sdf::ForceTorqueMeasureDirection measureDirection;
    sdf::ForceTorqueFrame measureFrame;
};
 
// Register plugin
GZ_ADD_PLUGIN(ForceTorque,
                    gz::sim::System,
                    ForceTorque::ISystemConfigure,
                    ForceTorque::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(ForceTorque, "gz::sim::systems::ForceTorque")