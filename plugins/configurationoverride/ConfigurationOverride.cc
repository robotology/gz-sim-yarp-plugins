#include <DeviceRegistry.hh>

#include <chrono>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

#include <thread>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>

using namespace gz;
using namespace sim;
using namespace systems;

using yarp::os::Bottle;
using yarp::os::BufferedPort;

namespace gzyarp
{

class ConfigurationOverride : public System, public ISystemConfigure, public ISystemPostUpdate
{
public:
    ConfigurationOverride(): m_overrideInserted(false)
    {
    }

    ~ConfigurationOverride()
    {
        // When the plugin is destroyed, we need to remove the override specified in the singleton,
        // otherwise it may happen that a new ECM (with exactly the same pointer of the old one) is created
        // and will find the overrides specified by the destroyed gzyarp::ConfigurationOverride plugin
        if (m_overrideInserted)
        {
            DeviceRegistry::getHandler()->removeConfigurationOverrideForYARPPlugin(m_configurationOverrideInstanceId);
            m_overrideInserted = false;
        }
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        if (!m_overrideInserted)
        {
            // We need to clone the sdf element to call non-const methods
            auto sdfClone = _sdf->Clone();

            gzyarp::PluginConfigureHelper configureHelper(_ecm);

            std::unordered_map<std::string, std::string> overridenParameters;

            if (!sdfClone->HasElement("yarpPluginConfigurationOverride"))
            {
                yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                            "missing yarpPluginConfigurationOverride element";
                return;
            }

            sdf::ElementPtr yarpPluginConfigurationOverrideElem = sdfClone->GetElement("yarpPluginConfigurationOverride");

            // There are two possible way of specifying an override:
            // * yarpDeviceName attribute, to specify that the override is related to a gz-sim-yarp-plugin that instantiates a YARP device
            // * yarpRobotInterfaceName attribute, to specify that the override is related to a gz-sim-robotinterface-plugin (at the moment only the `all` special value is supported)
            if (!yarpPluginConfigurationOverrideElem->HasAttribute("yarpDeviceName") && !yarpPluginConfigurationOverrideElem->HasAttribute("yarpRobotInterfaceName") )
            {
                yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                            "missing yarpDeviceName or yarpRobotInterfaceName attribute of yarpPluginConfigurationOverride element";
                return;
            }

            // Only one of yarpDeviceName or yarpRobotInterfaceName can be specified
            if (yarpPluginConfigurationOverrideElem->HasAttribute("yarpDeviceName") && yarpPluginConfigurationOverrideElem->HasAttribute("yarpRobotInterfaceName"))
            {
                yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                            "both yarpDeviceName and yarpRobotInterfaceName attributes of yarpPluginConfigurationOverride element are specified, while only one of the two is supported";
                return;
            }

            // Define unique key to identify the configuration override and then remove it
            std::stringstream ss;
            ss << this;
            m_configurationOverrideInstanceId = DeviceRegistry::getGzInstanceId(_ecm) + "_" + ss.str();

            // This code is used if yarpDeviceName is specified
            if (yarpPluginConfigurationOverrideElem->HasAttribute("yarpDeviceName"))
            {
                std::string yarpDeviceName = yarpPluginConfigurationOverrideElem->GetAttribute("yarpDeviceName")->GetAsString();

                if (sdfClone->HasElement("initialConfiguration"))
                {
                    overridenParameters["gzyarp-xml-element-initialConfiguration"] = sdfClone->Get<std::string>("initialConfiguration");
                }

                if (!DeviceRegistry::getHandler()->addConfigurationOverrideForYARPDevice(
                        _ecm,
                        scopedName(_entity, _ecm, "/"),
                        yarpDeviceName,
                        m_configurationOverrideInstanceId,
                        overridenParameters))
                {
                    yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                                "addConfigurationOverrideForYARPDevice failed";
                    return;
                }
            }

            // This code is used if yarpRobotInterfaceName is specified
            if (yarpPluginConfigurationOverrideElem->HasAttribute("yarpRobotInterfaceName"))
            {
                std::string yarpRobotInterfaceName = yarpPluginConfigurationOverrideElem->GetAttribute("yarpRobotInterfaceName")->GetAsString();

                if (yarpRobotInterfaceName != "all")
                {
                    yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                                "yarpRobotInterfaceName attribute of yarpPluginConfigurationOverride element must be 'all'";
                    return;
                }

                if (sdfClone->HasElement("yarpRobotInterfaceEnableTags"))
                {
                    overridenParameters["gzyarp-xml-element-yarpRobotInterfaceEnableTags"] = sdfClone->Get<std::string>("yarpRobotInterfaceEnableTags");
                }

                if (sdfClone->HasElement("yarpRobotInterfaceDisableTags"))
                {
                    overridenParameters["gzyarp-xml-element-yarpRobotInterfaceDisableTags"] = sdfClone->Get<std::string>("yarpRobotInterfaceDisableTags");
                }

                if (sdfClone->HasElement("yarpRobotInterfaceOverridePortPrefixWithModelName"))
                {
                    bool overrideValue = sdfClone->Get<bool>("yarpRobotInterfaceOverridePortPrefixWithModelName");
                    overridenParameters["gzyarp-xml-element-yarpRobotInterfaceOverridePortPrefixWithModelName"] = overrideValue ? "true" : "false";
                }

                if (!DeviceRegistry::getHandler()->addConfigurationOverrideForYARPRobotInterface(
                        _ecm,
                        scopedName(_entity, _ecm, "/"),
                        yarpRobotInterfaceName,
                        m_configurationOverrideInstanceId,
                        overridenParameters))
                {
                    yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                                "addConfigurationOverrideForYARPRobotInterface failed";
                    return;
                }
            }


            m_overrideInserted = true;
            configureHelper.setConfigureIsSuccessful(true);
        }
    }

    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
    }

private:
    bool m_overrideInserted;
    std::string m_configurationOverrideInstanceId;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::ConfigurationOverride,
              gz::sim::System,
              gzyarp::ConfigurationOverride::ISystemConfigure,
              gzyarp::ConfigurationOverride::ISystemPostUpdate)
