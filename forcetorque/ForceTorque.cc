#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/JointTransmittedWrench.hh"
#include "gz/sim/components/Pose.hh"

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
    auto model = Model(_entity);
    this->entity = _entity;
    this->joint = model.JointByName(_ecm, "joint_12");
    this->sensor = Joint(this->joint).SensorByName(_ecm, "force_torque");
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
    auto X_JS = _ecm.Component<components::Pose>(entity)->Data();

    math::Vector3d force = X_JS.Rot().Inverse() * msgs::Convert(jointWrench->Data().force());
    math::Vector3d torque = X_JS.Rot().Inverse() * msgs::Convert(jointWrench->Data().torque()) - X_JS.Pos().Cross(force);
    
    // Print force and torque values
    std::cout << _info.simTime.count()/1e9 << std::endl;
    std::cout << "Force:  " << force << std::endl;
    std::cout << "Torque: " << torque << std::endl << std::endl;
  }
 
  private: Entity sensor;
  private: Entity joint;
  private: Entity entity;
};
 
// Register plugin
GZ_ADD_PLUGIN(ForceTorque,
                    gz::sim::System,
                    ForceTorque::ISystemConfigure,
                    ForceTorque::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(ForceTorque, "gz::sim::systems::ForceTorque")