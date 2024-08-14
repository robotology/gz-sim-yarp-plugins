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

class DeviceRegistry
{
public:
    static DeviceRegistry* getHandler();

    bool getDevicesAsPolyDriverList(const gz::sim::EntityComponentManager& ecm,
                                    const std::string& modelScopedName,
                                    yarp::dev::PolyDriverList& list,
                                    std::vector<std::string>& deviceScopedNames) const;

    bool setDevice(const gz::sim::Entity& entity,
                   const gz::sim::EntityComponentManager& ecm,
                   const std::string& yarpDeviceName,
                   yarp::dev::PolyDriver* device2add,
                   std::string& generatedDeviceDatabaseKey);

    bool getDevice(const gz::sim::EntityComponentManager& ecm,
                   const std::string& deviceDatabaseKey,
                   yarp::dev::PolyDriver*& driver) const;

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

private:
    static std::string generateDeviceId(const gz::sim::Entity& entity,
                                        const gz::sim::EntityComponentManager& ecm,
                                        const std::string& yarpDeviceName);

    static std::string getGzInstanceId(const gz::sim::EntityComponentManager& ecm);

    static std::string getYarpDeviceName(const std::string& deviceDatabaseKey);

    static std::string getModelScopedName(const std::string& deviceDatabaseKey);

    DeviceRegistry();
    static DeviceRegistry* s_handle;
    static std::mutex& mutex();
    std::unordered_map<std::string, std::unordered_map<std::string, yarp::dev::PolyDriver*>>
        m_devicesMap; // map of known yarp devices Updated upstream
    // Event for when a device is removed
    gz::common::EventT<void (std::string)> m_deviceRemovedEvent;

    // Number of gz-sim-yarp-plugins YARP devices not loaded correctly for a given ecm
    std::unordered_map<std::string, std::size_t> m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded;
};

} // namespace gzyarp
