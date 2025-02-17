#pragma once

#include <gz/common/Event.hh>
#include <gz/common/events/Types.hh>
#include <gz/sim/Entity.hh>

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#include <gz/sim/EntityComponentManager.hh>

namespace gzyarp
{

/**
 * Class instantiated by all gz-sim-yarp-plugins in the plugin Configure.
 *
 * The class destructor calls DeviceRegistry::getHandler()->incrementNrOfGzSimYARPPluginsNotSuccessfullyLoaded(ecm),
 * unless the setConfigureIsSuccessful(true) method is called to signal that the configure of the plugin has been successful.
 * This is used to keep track of the number of gz-sim-yarp-plugins that have not been successfully loaded.
 */
class PluginConfigureHelper {
public:
    PluginConfigureHelper(const gz::sim::EntityComponentManager& ecm) : m_configureSuccessful(false), m_pecm(&ecm) {}
    ~PluginConfigureHelper();
    void setConfigureIsSuccessful(bool success);

private:
    bool m_configureSuccessful;
    const gz::sim::EntityComponentManager* m_pecm;
};

/**
 * Class that manages the YARP devices created by the gz-sim-yarp-plugins.
 *
 * The class is a singleton and is used to store the devices created by the gz-sim-yarp-plugins.
 */
class DeviceRegistry
{
public:
    // Return a pointer to the only global instance of the DeviceRegistry in the process
    static DeviceRegistry* getHandler();

    /**
     * Extract all the devices from the registry that correspond to a given gz server
     * and that belong to a model and all its descendants
     */
    bool getDevicesAsPolyDriverList(const gz::sim::EntityComponentManager& ecm,
                                    const std::string& modelScopedName,
                                    yarp::dev::PolyDriverList& list,
                                    std::vector<std::string>& deviceScopedNames) const;
    /**
     * Insert a device in the registry.
     *
     * This method MUST be called only by the gz plugin that created the YARP device
     */
    bool setDevice(const gz::sim::Entity& entity,
                   const gz::sim::EntityComponentManager& ecm,
                   const std::string& yarpDeviceName,
                   yarp::dev::PolyDriver* device2add,
                   std::string& generatedDeviceDatabaseKey);

    /**
     * Get a device whose device ID corresponds exactly to the specified deviceDatabaseKey.
     */
    bool getDevice(const gz::sim::EntityComponentManager& ecm,
                   const std::string& deviceDatabaseKey,
                   yarp::dev::PolyDriver*& driver) const;

    /**
     * Remove a device whose device ID corresponds exactly to the specified deviceDatabaseKey.
     */
    bool
    removeDevice(const gz::sim::EntityComponentManager& ecm, const std::string& deviceDatabaseKey);

    /**
     * Add a callback when a device is removed, i.e. removeDevice is called.
     */
    template<typename T>
    gz::common::ConnectionPtr connectDeviceRemoved(T _subscriber)
    {
        return m_deviceRemovedEvent.Connect(_subscriber);
    }

    /**
     * Get all the device ids (aka device database keys) of the devices in the registry for a given gz server.
     */
    std::vector<std::string> getDevicesKeys(const gz::sim::EntityComponentManager& ecm) const;

    /**
     * Get number of gz-sim-yarp-plugins not successfully loaded for a specific simulation server.
     */
    std::size_t getNrOfGzSimYARPPluginsNotSuccessfullyLoaded(const gz::sim::EntityComponentManager& ecm) const;

    /**
     * Get number of gz-sim-yarp-plugins not successfully loaded for all simulation in the process.
     */
    std::size_t getTotalNrOfGzSimYARPPluginsNotSuccessfullyLoaded() const;

    /**
     * Increment number of gz-sim-yarp-plugins not successfully loaded. This function is only meant to be called by
     * gz-sim-yarp-plugins gz-sim plugins.
     */
    void incrementNrOfGzSimYARPPluginsNotSuccessfullyLoaded(const gz::sim::EntityComponentManager& ecm);

    /**
     * Add a configuration override for a given yarp device .
     *
     * This function is meant to be called only by the gzyarp::ConfigurationOverride plugin.
     *
     */
    bool addConfigurationOverrideForYARPDevice(const gz::sim::EntityComponentManager& ecm,
                                               const std::string& parentEntityScopedNameWhereConfigurationOverrideWasInserted,
                                               const std::string& yarpDeviceName,
                                               const std::string& configurationOverrideInstanceId,
                                               std::unordered_map<std::string, std::string> overridenParameters);

    /**
     * Add a configuration override for a given yarp RobotInterface .
     *
     * This function is meant to be called only by the gzyarp::ConfigurationOverride plugin.
     *
     */
     bool addConfigurationOverrideForYARPRobotInterface(const gz::sim::EntityComponentManager& ecm,
        const std::string& parentEntityScopedNameWhereConfigurationOverrideWasInserted,
        const std::string& yarpRobotInterfaceName,
        const std::string& configurationOverrideInstanceId,
        std::unordered_map<std::string, std::string> overridenParameters);

    /**
     * Remove a configuration override for a gz-sim-yarp-plugin.
     *
     */
    bool removeConfigurationOverrideForYARPPlugin(const std::string& configurationOverrideInstanceId);

    /**
     * Get the configuration override for a given yarp device.
     *
     * This function is meant to be called by each plugin to override its configuration.
     *
     */
    bool getConfigurationOverrideForYARPDevice(const gz::sim::EntityComponentManager& ecm,
                                               const std::string& parentEntityScopedNameWhereYARPDeviceWasInserted,
                                               const std::string& yarpDeviceName,
                                               std::unordered_map<std::string, std::string>& overridenParameters) const;

    /**
     * Get the configuration override for a given yarpRobotInterfaceName.
     *
     * This function is meant to be called by the gz-yarp-robotinterface plugin to override its configuration.
     *
     * At the moment only the `all` yarpRobotInterfaceName special value (to represent all the robotinterface in the nested models) is supported.
     */
     bool getConfigurationOverrideForYARPRobotInterface(const gz::sim::EntityComponentManager& ecm,
        const std::string& parentEntityScopedNameWhereYARPRobotInterfaceWasInserted,
        const std::string& yarpRobotInterfaceName,
        std::unordered_map<std::string, std::string>& overridenParameters) const;


    /**
     * Generate a unique device id for a given yarp device name and entity.
     *
     * The device id is a process-unique string that identifies a device in a given gz server.
     * Inside the process, a device is uniquely identified by the pair of the gz instance id and the device id.
     */
     static std::string generateDeviceId(const gz::sim::Entity& parentEntity,
        const gz::sim::EntityComponentManager& ecm,
        const std::string& yarpDeviceName);

    /**
     * Generate a unique instance id for a given gz server.
     */
    static std::string getGzInstanceId(const gz::sim::EntityComponentManager& ecm);

    /**
    * Extract the yarpDeviceName from a device id, i.e. the third argument passed to generateDeviceId.
    */
    static std::string getYarpDeviceName(const std::string& deviceId);

    /**
    * Extract the scoped name of the parent entity from a device id, i.e. the scoped name of the parent entity of the plugin that created the device.
    */
    static std::string getParentEntityScopedName(const std::string& deviceId);

private:
    // Private constructor. The constructor is private to ensure that the class is a singleton.
    DeviceRegistry();

    // Static instance of the DeviceRegistry
    static DeviceRegistry* s_handle;
    static std::mutex& mutex();

    // Map that stores all the yarp devices created by the gz-sim-yarp-plugins
    // The key of the first map is the gz instance id, the key of the second map is the device id
    std::unordered_map<std::string, std::unordered_map<std::string, yarp::dev::PolyDriver*>>
        m_devicesMap;

    // Event for when a device is removed
    gz::common::EventT<void (std::string)> m_deviceRemovedEvent;

    // Number of gz-sim-yarp-plugins YARP devices not loaded correctly for a given ecm
    std::unordered_map<std::string, std::size_t> m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded;

    enum OverrideType {
        YARP_DEVICE,
        YARP_ROBOT_INTERFACE
    };

    // Element that stores the parameters that will be overriden for a given yarp device or robotinterface
    // A plugin will be affected by the override if:
    // - gzInstanceId match
    // - (overrideType,yarpPluginIdentifier) match
    // - the parentEntityScopedNameWhereConfigurationOverrideWasInserted is a prefix of the parent entity scoped name of the device`
    struct ConfigurationOverrideParameters {
        // Id that identifies the gz instance where the configuration override plugin was inserted
        std::string gzInstanceId;
        // Scoped name of the parent entity where the condiguration override plugin was inserted,
        // used to understand if a given yarp device is affected by the override
        std::string parentEntityScopedNameWhereConfigurationOverrideWasInserted;
        // Specify if this is a override for a device or a robotinterface
        OverrideType overrideType;
        // This is yarpDeviceName if overrideType==YARP_DEVICE or yarpRobotInterfaceName if overrideType==YARP_ROBOT_INTERFACE
        std::string yarpPluginIdentifier;
        // configurationOverrideInstanceId is a unique id for each element in the m_yarpPluginsOverridenParametersList,
        // it is used to remove a configuration override when the corresponding plugin is destroyed
        std::string configurationOverrideInstanceId;
        // Map of the parameters that will be overriden
        // Some keys have a special meaning:
        // gzyarp-xml-element-initialConfiguration: override the content of a <initialConfiguration> tag
        std::unordered_map<std::string, std::string> overridenParameters;
    };

    // This is a list of parameters specified by the configuration override plugin,
    // they specify the parameters that will be overriden for a given yarp plugin (device or robotinterface)
    std::vector<ConfigurationOverrideParameters> m_yarpPluginsOverridenParametersList;
};

} // namespace gzyarp
