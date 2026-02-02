#include <DeviceRegistry.hh>
#include <gzyarp/ConfigurationHelpers.hh>

#include <functional>
#include <memory>
#include <sdf/Element.hh>
#include <string>
#include <vector>

#include <gz/common/Event.hh>
#include <gz/common/events/Types.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/robotinterface/Types.h>
#include <yarp/robotinterface/XMLReader.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{

class RobotInterface : public System, public ISystemConfigure
{
public:
    RobotInterface()
        : m_robotInterfaceCorrectlyStarted(false)
    {
    }

    virtual ~RobotInterface()
    {
        CloseRobotInterface();
        yarp::os::Network::fini();
    }

    void CloseRobotInterface()
    {
        if (m_robotInterfaceCorrectlyStarted)
        {
            // Close robotinterface
            bool ok = m_xmlRobotInterfaceResult.robot.enterPhase(
                yarp::robotinterface::ActionPhaseInterrupt1);
            if (!ok)
            {
                yError() << "gz-sim-yarp-robotinterface-system: impossible to run phase "
                            "ActionPhaseInterrupt1 robotinterface";
            }
            ok = m_xmlRobotInterfaceResult.robot.enterPhase(
                yarp::robotinterface::ActionPhaseShutdown);
            if (!ok)
            {
                yError() << "gz-sim-yarp-robotinterface-system: impossible  to run phase "
                            "ActionPhaseShutdown in robotinterface";
            }
            m_deviceRemovedConnection.reset();
            m_clockPluginRemovedConnection.reset();
            m_robotInterfaceCorrectlyStarted = false;
        }
    }

    void OnDeviceRemoved(std::string gzInstanceId, std::string removeDeviceRegistryDatabaseKey)
    {
        // If the device removed is not in the same gz instance of the robotinterface, return
        if (gzInstanceId != m_gzInstanceId)
        {
            return;
        }

        // Check if deviceRegistryDatabaseKey is among the one passed to this instance of gz_yarp_robotinterface
        // If yes, close the robotinterface to avoid crashes due to access to a device that is being deleted
        for (auto&& usedDeviceScopedName: m_deviceScopedNames) {
            if (removeDeviceRegistryDatabaseKey == usedDeviceScopedName) {
                CloseRobotInterface();
            }
        }
        return;
    }

    void OnClockPluginRemoved(std::string gzInstanceId, std::string removedClockPluginID)
    {
        // If the device removed is not in the same gz instance of the robotinterface, return
        if (gzInstanceId != m_gzInstanceId)
        {
            return;
        }

        // If the clock plugin belongs to the same gz instance of the robotinterface and belongs
        // to a ancestor entity, close the robotinterface
        bool closeRobotInterface = false;
        // Workaround for https://github.com/robotology/gz-sim-yarp-plugins/pull/253#issuecomment-2730881727
#if defined(GZ_SIM_MAJOR_VERSION) && (GZ_SIM_MAJOR_VERSION >= 9)
        std::string parentEntityScopedNameWhereClockPluginWasInserted = DeviceRegistry::getParentEntityScopedName(removedClockPluginID);
        closeRobotInterface = (m_parentEntityScopedName.rfind(parentEntityScopedNameWhereClockPluginWasInserted) == 0);
#else
        // Always close the robotinterface if the clock plugin is removed on gz-sim <= 8
        closeRobotInterface = true;
#endif
        if (closeRobotInterface)
        {
            CloseRobotInterface();
        }
        return;
    }


    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        yarp::os::Network::init();
        auto model = Model(_entity);

        gzyarp::PluginConfigureHelper configureHelper(_ecm);

        if (!loadYarpRobotInterfaceConfigurationFile(_sdf, _ecm, model, _entity))
        {
            yError("gz-sim-yarp-robotinterface-system : Error loading robotinterface configuration "
                   "file");
            return;
        }

        yarp::dev::PolyDriverList externalDriverList;
        DeviceRegistry::getHandler()->getDevicesAsPolyDriverList(_ecm,
                                                                 scopedName(model.Entity(), _ecm),
                                                                 externalDriverList,
                                                                 m_deviceScopedNames);

        bool ok = m_xmlRobotInterfaceResult.robot.setExternalDevices(externalDriverList);
        if (!ok)
        {
            yError() << "gz-sim-yarp-robotinterface-system : impossible to set external devices";
            return;
        }

        // Start robotinterface
        ok = m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup);
        if (!ok)
        {
            yError() << "gz-sim-yarp-robotinterface-system : impossible to start robotinterface";
            m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt1);
            m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseShutdown);
            return;
        }

        configureHelper.setConfigureIsSuccessful(true);
        m_robotInterfaceCorrectlyStarted = true;

        // If the robotinterface started correctly, add a callback to ensure that it is closed as
        // soon that an external device passed to it is deleted
        m_deviceRemovedConnection =
            DeviceRegistry::getHandler()->connectDeviceRemoved(
                std::bind(&RobotInterface::OnDeviceRemoved, this, std::placeholders::_1,  std::placeholders::_2));

        // If the robotinterface started correctly, add a callback to ensure that it is closed as
        // soon that the clock plugin is deleted
        m_clockPluginRemovedConnection =
            DeviceRegistry::getHandler()->connectClockPluginRemoved(
                std::bind(&RobotInterface::OnClockPluginRemoved, this, std::placeholders::_1,  std::placeholders::_2));
    }

private:
    yarp::robotinterface::XMLReaderResult m_xmlRobotInterfaceResult;
    std::vector<std::string> m_deviceScopedNames;
    gz::common::ConnectionPtr m_deviceRemovedConnection;
    gz::common::ConnectionPtr m_clockPluginRemovedConnection;
    bool m_robotInterfaceCorrectlyStarted;
    std::string m_yarpRobotInterfaceName;
    std::string m_parentEntityScopedName;
    std::string m_gzInstanceId;
    bool m_yarpRobotInterfaceOverridePortPrefixWithModelName;

    gz::sim::Entity FindModel(gz::sim::Entity entity,  const gz::sim::EntityComponentManager &ecm)
    {
        while (entity != gz::sim::kNullEntity)
        {
            if (ecm.Component<gz::sim::components::Model>(entity)) { return entity; }

            auto parent =  ecm.Component<gz::sim::components::ParentEntity>(entity);
            if (!parent) { break; }

            entity = parent->Data();
        }
        return gz::sim::kNullEntity;
    }

    gz::sim::Entity FindTopLevelModel(gz::sim::Entity entity,  const gz::sim::EntityComponentManager &ecm)
    {
        gz::sim::Entity topModelEntity = gz::sim::kNullEntity;

        while (entity != gz::sim::kNullEntity)
        {
            if (ecm.Component<gz::sim::components::Model>(entity))
            {
                topModelEntity = entity;
            }

            auto parent = ecm.Component<gz::sim::components::ParentEntity>(entity);
            if (!parent) { break; }

            entity = parent->Data();
        }
        return topModelEntity;
    }

    bool loadYarpRobotInterfaceConfigurationFile(const std::shared_ptr<const sdf::Element>& _sdf,
                                                 const EntityComponentManager& _ecm,
                                                 const Model& model, const Entity& _entity)
    {
        if (!_sdf->HasElement("yarpRobotInterfaceConfigurationFile"))
        {
            yError() << "gz-sim-yarp-robotinterface-system :"
                        "yarpRobotInterfaceConfigurationFile element not found";

            return false;
        }

        if (!_sdf->HasElement("yarpRobotInterfaceConfigurationFile"))
        {
            yError() << "yarpRobotInterfaceConfigurationFile element not found";
            return false;
        }

        auto robotinterface_file_name = _sdf->Get<std::string>("yarpRobotInterfaceConfigurationFile");
        std::string filepath;
        if (!ConfigurationHelpers::findFile(robotinterface_file_name, filepath))
        {
            yError() << "Error while finding yarpRobotInterfaceConfigurationFile: "
                     << robotinterface_file_name;
            return false;
        }

        yarp::robotinterface::XMLReader xmlRobotInterfaceReader;

        // We read the file once just to read the name of the robotinterface, so that we can process the override
        m_xmlRobotInterfaceResult = xmlRobotInterfaceReader.getRobotFromFile(filepath);
        m_yarpRobotInterfaceName = m_xmlRobotInterfaceResult.robot.name();

        // Now we read the enable and disable tags specified in the SDF, and if available the overrides
        std::string enableTags = "";
        std::string disableTags = "";
        std::string prefixTag = "";
        bool overridePortPrefix = false;
        if (_sdf->HasElement("yarpRobotInterfaceEnableTags"))
        {
            enableTags = _sdf->Get<std::string>("yarpRobotInterfaceEnableTags");
        }
        if (_sdf->HasElement("yarpRobotInterfaceDisableTags"))
        {
            disableTags = _sdf->Get<std::string>("yarpRobotInterfaceDisableTags");
        }
        if (_sdf->HasElement("yarpRobotInterfaceOverridePortPrefixWithModelName"))
        {
            overridePortPrefix = _sdf->Get<bool>("yarpRobotInterfaceOverridePortPrefixWithModelName");
        }

        gz::sim::Entity modelEntity = FindTopLevelModel(_entity,_ecm);
        std::string modelName = _ecm.Component<gz::sim::components::Name>(modelEntity)->Data();

        // Then check if there is any override for the enable and disable tags elements
        m_gzInstanceId = DeviceRegistry::getHandler()->getGzInstanceId(_ecm);
        m_parentEntityScopedName = gz::sim::scopedName(_entity, _ecm, "/");
        std::unordered_map<std::string, std::string> overridenParameters;
        DeviceRegistry::getHandler()->getConfigurationOverrideForYARPRobotInterface(_ecm, m_parentEntityScopedName, m_yarpRobotInterfaceName, overridenParameters);

        if (overridenParameters.find("gzyarp-xml-element-yarpRobotInterfaceEnableTags") != overridenParameters.end())
        {
            enableTags = overridenParameters["gzyarp-xml-element-yarpRobotInterfaceEnableTags"];
        }
        if (overridenParameters.find("gzyarp-xml-element-yarpRobotInterfaceDisableTags") != overridenParameters.end())
        {
            disableTags = overridenParameters["gzyarp-xml-element-yarpRobotInterfaceDisableTags"];
        }
        if (overridenParameters.find("gzyarp-xml-element-yarpRobotInterfaceOverridePortPrefixWithModelName") != overridenParameters.end())
        {
            std::string overridePortPrefixStr = overridenParameters["gzyarp-xml-element-yarpRobotInterfaceOverridePortPrefixWithModelName"];
            overridePortPrefix = (overridePortPrefixStr == "true" || overridePortPrefixStr == "1");
        }

        // Store the flag in member variable
        m_yarpRobotInterfaceOverridePortPrefixWithModelName = overridePortPrefix;

        // Only override the port prefix with model name if the flag is true
        if (overridePortPrefix)
        {
            prefixTag = modelName;
            if (!prefixTag.empty() && prefixTag[0] != '/') {prefixTag.insert(prefixTag.begin(), '/');}
        }

        yarp::os::Property config;
        if (!enableTags.empty())
        {
            yarp::os::Bottle bot;
            bot.fromString(enableTags);
            config.put("enable_tags", bot.get(0));
        }
        if (!disableTags.empty())
        {
            yarp::os::Bottle bot;
            bot.fromString(disableTags);
            config.put("disable_tags", bot.get(0));
        }
        if (!prefixTag.empty())
        {
            yarp::os::Bottle bot;
            bot.fromString(prefixTag);
            config.put("portprefix", bot.get(0));
        }

        m_xmlRobotInterfaceResult = xmlRobotInterfaceReader.getRobotFromFile(filepath, config);

        if (!m_xmlRobotInterfaceResult.parsingIsSuccessful)
        {
            yError() << "Failure in parsing yarpRobotInterfaceConfigurationFile: "
                     << robotinterface_file_name;
            return false;
        }


        return true;
    }
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::RobotInterface, gz::sim::System, gzyarp::RobotInterface::ISystemConfigure)
