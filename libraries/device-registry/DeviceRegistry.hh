#pragma once

#include <gz/common/Event.hh>
#include <gz/sim/Entity.hh>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#include <gz/sim/EntityComponentManager.hh>

namespace gzyarp
{

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
};

} // namespace gzyarp
