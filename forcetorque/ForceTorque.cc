#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Sensor.hh"


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

    auto joint = model.JointByName(_ecm, "joint_12");
    this->sensor = _ecm.EntityByComponents(
      components::ParentEntity(joint),
      components::Name("force_torque"),
      components::Sensor());
  }
 
  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  virtual void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override
  {
    std::string name = _ecm.ComponentData<components::Name>(this->sensor).value();
    auto parent = _ecm.Component<components::ParentEntity>(this->sensor)->Data();
    std::string parentName = _ecm.ComponentData<components::Name>(parent).value();
    std::cout << "Sensor name: "+ name + ", Parent name: " + parentName << std::endl;
  }
 
  // ID sensor entity
  private: Entity sensor;
};
 
// Register plugin
GZ_ADD_PLUGIN(ForceTorque,
                    gz::sim::System,
                    ForceTorque::ISystemConfigure,
                    ForceTorque::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(ForceTorque, "gz::sim::systems::ForceTorque")