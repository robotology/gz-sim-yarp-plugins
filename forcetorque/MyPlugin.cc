#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
 
using namespace gz;
using namespace sim;
using namespace systems;
 
// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class MyPlugin
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
    // Get link entity
    this->linkEntity = model.LinkByName(_ecm, "link_2");
  }
 
  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  virtual void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override
  {
    // Get link pose and print it
    std::cout << worldPose(this->linkEntity, _ecm) << std::endl;

    // This is a simple example of how to get information from UpdateInfo.
    std::string msg = "Simulation is ";
    if (!_info.paused)
      msg += "not ";
    msg += "paused.";
    // Messages printed with gzmsg only show when running with verbosity 3 or
    // higher (i.e. gz sim -v 3)
    gzmsg << msg << std::endl;
  }
 
  // ID of link entity
  private: Entity linkEntity;
};
 
// Register plugin
GZ_ADD_PLUGIN(MyPlugin,
                    gz::sim::System,
                    MyPlugin::ISystemConfigure,
                    MyPlugin::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(MyPlugin, "gz::sim::systems::MyPlugin")