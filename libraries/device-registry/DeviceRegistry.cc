#include <DeviceRegistry.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include <cstddef>
#include <iostream>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gzyarp
{

PluginConfigureHelper::~PluginConfigureHelper()
{
    if (!m_configureSuccessful)
    {
        DeviceRegistry::getHandler()->incrementNrOfGzSimYARPPluginsNotSuccessfullyLoaded(*m_pecm);
    }
}

void PluginConfigureHelper::setConfigureIsSuccessful(bool success)
{
    m_configureSuccessful = success;
    return;
}

DeviceRegistry* DeviceRegistry::getHandler()
{
    std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle)
    {
        s_handle = new DeviceRegistry();
        if (!s_handle)
            yError() << "Error while calling gzyarp::DeviceRegistry constructor";
    }

    return s_handle;
}

bool DeviceRegistry::getDevicesAsPolyDriverList(const gz::sim::EntityComponentManager& ecm,
                                                const std::string& parentEntityScopedName,
                                                yarp::dev::PolyDriverList& list,
                                                std::vector<std::string>& deviceScopedNames) const
{
    deviceScopedNames.resize(0);
    list = yarp::dev::PolyDriverList();

    {
        std::lock_guard<std::mutex> lock(mutex());

        // Check if the the gz instance has been already added to the map
        std::string gzInstanceId = getGzInstanceId(ecm);
        if (auto gzInstance_it = m_devicesMap.find(gzInstanceId);
            gzInstance_it == m_devicesMap.end())
        {
            // If no instances is found, it probably means that getDevicesAsPolyDriverList is called
            // for a gz instance that has not created any device yet, so just returning an empty list
            // is the correct way to handle this case
            return true;
        }

        auto& devicesMap = m_devicesMap.at(gzInstanceId);

        for (auto&& [key, value] : devicesMap)
        {
            std::string deviceParentEntityScopedName = getParentEntityScopedName(key);
            std::string yarpDeviceName = getYarpDeviceName(key);

            if (deviceParentEntityScopedName == parentEntityScopedName)
            {
                list.push(value, yarpDeviceName.c_str());
                deviceScopedNames.push_back(key);
            }
        }
    }
    return true;
}

bool DeviceRegistry::setDevice(const gz::sim::Entity& entity,
                               const gz::sim::EntityComponentManager& ecm,
                               const std::string& yarpDeviceName,
                               yarp::dev::PolyDriver* device2add,
                               std::string& generatedDeviceDatabaseKey)
{
    bool ret = false;
    generatedDeviceDatabaseKey = "";

    if (!device2add)
    {
        yError() << "Error in gzyarp::DeviceRegistry::setDevice: device2add is nullptr";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex());

        // Check if the the gz instance has been already added to the map
        std::string gzInstanceId = getGzInstanceId(ecm);
        if (auto gzInstance_it = m_devicesMap.find(gzInstanceId);
            gzInstance_it == m_devicesMap.end())
        {
            // If not, add it
            m_devicesMap.insert(
                std::pair<std::string, std::unordered_map<std::string, yarp::dev::PolyDriver*>>(
                    gzInstanceId, std::unordered_map<std::string, yarp::dev::PolyDriver*>{}));
        }

        // Check if the device has been already added to the map for the gz instance
        std::string deviceDatabaseKey = generateDeviceId(entity, ecm, yarpDeviceName);
        auto device_it = m_devicesMap[gzInstanceId].find(deviceDatabaseKey);
        if (device_it == m_devicesMap[gzInstanceId].end())
        {
            // If not, add it
            m_devicesMap[gzInstanceId].insert(
                std::pair<std::string, yarp::dev::PolyDriver*>(deviceDatabaseKey, device2add));
            generatedDeviceDatabaseKey = deviceDatabaseKey;

            ret = true;
        } else
        {
            if (device_it->second == device2add)
            {
                generatedDeviceDatabaseKey = deviceDatabaseKey;
                ret = true;
            } else
            {

                yError() << " Error in gzyarp::DeviceRegistry while inserting a new yarp "
                            "device "
                            "pointer!";
                yError() << " The name of the device is already present but the pointer does "
                            "not match "
                            "with the one already registered!";
                yError() << " This should not happen, check the names are correct in your "
                            "config file. "
                            "The device already in the map has parent entity scoped name: "
                         << getParentEntityScopedName(device_it->first)
                         << " and yarp device name: " << getYarpDeviceName(device_it->first)
                         << ". "
                            "The device you are trying to insert has parent entity scoped name: "
                         << getParentEntityScopedName(deviceDatabaseKey)
                         << " and yarp device name: " << getYarpDeviceName(deviceDatabaseKey)
                         << ". "
                            "Fatal error.";
                ret = false;
            }
        }
    }

    return ret;
}

bool DeviceRegistry::getDevice(const gz::sim::EntityComponentManager& ecm,
                               const std::string& deviceDatabaseKey,
                               yarp::dev::PolyDriver*& driver) const
{
    driver = nullptr;

    {
        std::lock_guard<std::mutex> lock(mutex());

        // Check if the the gz instance exists the map
        std::string gzInstanceId = getGzInstanceId(ecm);
        if (auto gzInstance_it = m_devicesMap.find(gzInstanceId);
            gzInstance_it == m_devicesMap.end())
        {
            yError() << "Error in gzyarp::DeviceRegistry::getDevice: gz instance not found";
            return false;
        }

        // Check if the device exists in the map
        if (auto device_it = m_devicesMap.at(gzInstanceId).find(deviceDatabaseKey);
            device_it == m_devicesMap.at(gzInstanceId).end())
        {
            yError() << "Error in gzyarp::DeviceRegistry::getDevice: device not found";
            return false;
        }

        driver = m_devicesMap.at(gzInstanceId).at(deviceDatabaseKey);

        if (!driver)
        {
            yError() << "Error in gzyarp::DeviceRegistry::getDevice: driver is "
                        "nullptr";
            return false;
        }
    }

    return true;
}

std::vector<std::string>
DeviceRegistry::getDevicesKeys(const gz::sim::EntityComponentManager& ecm) const
{
    std::vector<std::string> keys{};

    {
        std::lock_guard<std::mutex> lock(mutex());

        // Check if the the gz instance exists the map
        std::string gzInstanceId = getGzInstanceId(ecm);
        if (auto gzInstance_it = m_devicesMap.find(gzInstanceId);
            gzInstance_it == m_devicesMap.end())
        {
            yError() << "Error in gzyarp::DeviceRegistry::getDevicesKeys: gz instance not found";
            return keys;
        }

        for (auto&& [key, value] : m_devicesMap.at(gzInstanceId))
        {
            keys.push_back(key);
        }
    }

    return keys;
}

bool DeviceRegistry::removeDevice(const gz::sim::EntityComponentManager& ecm,
                                  const std::string& deviceDatabaseKey)
{

    {
        std::lock_guard<std::mutex> lock(mutex());

        // Check if the the gz instance exists the map
        std::string gzInstanceId = getGzInstanceId(ecm);
        if (auto gzInstance_it = m_devicesMap.find(gzInstanceId);
            gzInstance_it == m_devicesMap.end())
        {
            yError() << "Error in gzyarp::DeviceRegistry::getDevicesKeys: gz instance not found";
            return false;
        }

        // Check if the device exists in the map
        if (auto device_it = m_devicesMap.at(gzInstanceId).find(deviceDatabaseKey);
            device_it == m_devicesMap.at(gzInstanceId).end())
        {
            yError() << "Error in gzyarp::DeviceRegistry::getDevice: device not found";
            return false;
        } else
        {
            if (!device_it->second->close())
            {
                yError() << "Error in gzyarp::DeviceRegistry::removeDevice: device could not be "
                            "closed";
                return false;
            }

            m_deviceRemovedEvent(gzInstanceId, deviceDatabaseKey);
            m_devicesMap.at(gzInstanceId).erase(deviceDatabaseKey);
        }
    }
    return true;
}

bool DeviceRegistry::insertClockPlugin(const gz::sim::Entity& entity,
    const gz::sim::EntityComponentManager& ecm,
          std::string& generatedClockPluginID)
{
    bool ret = false;
    generatedClockPluginID = "";

    {
        std::lock_guard<std::mutex> lock(mutex());

        // Check if the the gz instance has been already added to the map
        std::string gzInstanceId = getGzInstanceId(ecm);
        if (auto gzInstance_it = m_clockPluginsMap.find(gzInstanceId);
            gzInstance_it == m_clockPluginsMap.end())
        {
            // If not, insert an empty vector in the map
            m_clockPluginsMap.insert(std::pair<std::string, std::vector<std::string>>(gzInstanceId, std::vector<std::string>{}));
        }

        // Check if the device has been already added to the map for the gz instance
        // We reuse the logic for the yarp device id generation, but as in the case of the plugin
        // we do not have a yarpDeviceName, we just use an arbitrary string as placeholder
        std::string clockPluginID = generateDeviceId(entity, ecm, "clockPluginPlaceholderInPlaceOfYarpDeviceName");
        std::vector<std::string>& vec = m_clockPluginsMap[gzInstanceId];

        // Check if clockPluginID is contained in vec
        auto clockPlugin_it = std::find(vec.begin(), vec.end(), clockPluginID);
        if (clockPlugin_it == m_clockPluginsMap[gzInstanceId].end())
        {
            // If not, add it
            m_clockPluginsMap[gzInstanceId].push_back(clockPluginID);
            generatedClockPluginID = clockPluginID;

            ret = true;
        } else
        {
            yError() << " Error, two clock plugins in the same world have been found";
            yError() << " This should not happen, check you do not have a duplicated clock in your config file. ";
            ret = false;
        }
    }

    return ret;
}

bool DeviceRegistry::removeClockPlugin(const gz::sim::EntityComponentManager& ecm, const std::string& generatedClockPluginID)
{
    {
        std::lock_guard<std::mutex> lock(mutex());

        // Check if the the gz instance exists the map
        std::string gzInstanceId = getGzInstanceId(ecm);
        if (auto gzInstance_it = m_clockPluginsMap.find(gzInstanceId);
            gzInstance_it == m_clockPluginsMap.end())
        {
            yError() << "Error in gzyarp::DeviceRegistry::removeClockPlugin: gz instance not found";
            return false;
        }

        // Check if the device exists in the map
        std::vector<std::string>& vec = m_clockPluginsMap.at(gzInstanceId);
        auto clockPlugin_it = std::find(vec.begin(), vec.end(), generatedClockPluginID);
        if (clockPlugin_it == vec.end())
        {
            yError() << "Error in gzyarp::DeviceRegistry::removeClockPlugin: clockPlugin not found";
            return false;
        } else
        {
            m_clockPluginRemovedEvent(gzInstanceId, generatedClockPluginID);
            std::vector<std::string>& vec = m_clockPluginsMap[gzInstanceId];
            vec.erase(std::remove(vec.begin(), vec.end(), generatedClockPluginID), vec.end());
        }
    }
    return true;
}

// Private methods

std::string DeviceRegistry::generateDeviceId(const gz::sim::Entity& entity,
                                             const gz::sim::EntityComponentManager& ecm,
                                             const std::string& yarpDeviceName)
{
    auto scopedName = gz::sim::scopedName(entity, ecm, "/");
    return scopedName + "/" + yarpDeviceName;
}

std::string DeviceRegistry::getGzInstanceId(const gz::sim::EntityComponentManager& ecm)
{
    auto ecmPtr = &ecm;
    std::stringstream ss;
    ss << ecmPtr;
    return ss.str();
}

std::string DeviceRegistry::getYarpDeviceName(const std::string& deviceDatabaseKey)
{
    // Extract yarpDeviceName from deviceDatabaseKey
    std::string yarpDeviceName = deviceDatabaseKey.substr(deviceDatabaseKey.find_last_of("/") + 1);
    return yarpDeviceName;
}

std::string DeviceRegistry::getParentEntityScopedName(const std::string& deviceDatabaseKey)
{
    // Extract parentEntityScopedName from deviceDatabaseKey
    std::string parentEntityScopedName = deviceDatabaseKey.substr(0, deviceDatabaseKey.find_last_of("/"));
    return parentEntityScopedName;
}

DeviceRegistry::DeviceRegistry()
    : m_devicesMap()
{
    m_devicesMap.clear();
}

DeviceRegistry* DeviceRegistry::s_handle = NULL;

std::mutex& DeviceRegistry::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}

std::size_t DeviceRegistry::getNrOfGzSimYARPPluginsNotSuccessfullyLoaded(const gz::sim::EntityComponentManager& ecm) const
{
    auto it = m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded.find(getGzInstanceId(ecm));

    if (it == m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded.end())
    {
        return 0;
    } else
    {
        return it->second;
    }
}

std::size_t DeviceRegistry::getTotalNrOfGzSimYARPPluginsNotSuccessfullyLoaded() const
{
    std::size_t ret = 0;
    for (const auto& pair : m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded) {
        ret += pair.second;
    }
    return ret;
}

void DeviceRegistry::incrementNrOfGzSimYARPPluginsNotSuccessfullyLoaded(const gz::sim::EntityComponentManager& ecm)
{
    std::string ecmid = getGzInstanceId(ecm);
    auto it = m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded.find(getGzInstanceId(ecm));

    if (it == m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded.end())
    {
        m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded[ecmid] = 1;
    }
    else
    {
        m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded[ecmid] = m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded[ecmid] + 1;
    }
}

bool DeviceRegistry::addConfigurationOverrideForYARPDevice(const gz::sim::EntityComponentManager& ecm,
                                           const std::string& parentEntityScopedNameWhereConfigurationOverrideWasInserted,
                                           const std::string& yarpDeviceName,
                                           const std::string& configurationOverrideInstanceId,
                                           std::unordered_map<std::string, std::string> overridenParameters)
{
    m_yarpPluginsOverridenParametersList.push_back({getGzInstanceId(ecm),
                                                    parentEntityScopedNameWhereConfigurationOverrideWasInserted,
                                                    OverrideType::YARP_DEVICE,
                                                    yarpDeviceName,
                                                    configurationOverrideInstanceId,
                                                    overridenParameters});
    return true;
}

bool DeviceRegistry::addConfigurationOverrideForYARPRobotInterface(const gz::sim::EntityComponentManager& ecm,
    const std::string& parentEntityScopedNameWhereConfigurationOverrideWasInserted,
    const std::string& yarpRobotInterfaceName,
    const std::string& configurationOverrideInstanceId,
    std::unordered_map<std::string, std::string> overridenParameters)
{
    m_yarpPluginsOverridenParametersList.push_back({getGzInstanceId(ecm),
             parentEntityScopedNameWhereConfigurationOverrideWasInserted,
             OverrideType::YARP_ROBOT_INTERFACE,
             yarpRobotInterfaceName,
             configurationOverrideInstanceId,
             overridenParameters});
    return true;
}


bool DeviceRegistry::removeConfigurationOverrideForYARPPlugin(const std::string& configurationOverrideInstanceId)
{
    m_yarpPluginsOverridenParametersList.erase(
        std::remove_if(
            m_yarpPluginsOverridenParametersList.begin(),
            m_yarpPluginsOverridenParametersList.end(),
            [&configurationOverrideInstanceId](const ConfigurationOverrideParameters& params) {
                return params.configurationOverrideInstanceId == configurationOverrideInstanceId;
            }),
            m_yarpPluginsOverridenParametersList.end());
    return true;
}

bool DeviceRegistry::getConfigurationOverrideForYARPDevice(const gz::sim::EntityComponentManager& ecm,
                                           const std::string& parentEntityScopedNameWhereYARPDeviceWasInserted,
                                           const std::string& yarpDeviceName,
                                           std::unordered_map<std::string, std::string>& overridenParameters) const
{
    overridenParameters.clear();
    // We go through all the overriden parameters and add to the returned overridenParameters all the matchin
    // elements of the list. Note that earlier elements in the list have higher priority, to provide the possibility
    // of overriding a parameter that was already overriden in a nested configuration override.
    // Note that the condition for the overriden to be consider (beside matching gzInstanceId and yarpDeviceName)
    // is that the model scoped name of the place where the configuration override was inserted is a prefix of the
    // model scoped name of the device, to ensure that the override is applied only to the devices that are descendent
    // of the model where the configuration override was inserted
    for (auto&& params : m_yarpPluginsOverridenParametersList) {
        if (params.gzInstanceId == getGzInstanceId(ecm) &&
            parentEntityScopedNameWhereYARPDeviceWasInserted.rfind(params.parentEntityScopedNameWhereConfigurationOverrideWasInserted) == 0 &&
            params.overrideType == OverrideType::YARP_DEVICE &&
            (params.yarpPluginIdentifier == yarpDeviceName || params.yarpPluginIdentifier == "all")) {
            std::unordered_map<std::string, std::string> consideredOverridenParameters = params.overridenParameters;
            overridenParameters.merge(consideredOverridenParameters);
        }
    }

    return true;
}

bool DeviceRegistry::getConfigurationOverrideForYARPRobotInterface(const gz::sim::EntityComponentManager& ecm,
    const std::string& parentEntityScopedNameWhereYARPRobotInterfaceWasInserted,
    const std::string& yarpRobotInterfaceName,
    std::unordered_map<std::string, std::string>& overridenParameters) const
{
    overridenParameters.clear();
    // We go through all the overriden parameters and add to the returned overridenParameters all the matching
    // elements of the list. Note that earlier elements in the list have higher priority, to provide the possibility
    // of overriding a parameter that was already overriden in a nested configuration override.
    // Note that the condition for the overriden to be consider (beside matching gzInstanceId and yarpDeviceName)
    // is that the model scoped name of the place where the configuration override was inserted is a prefix of the
    // model scoped name of the device, to ensure that the override is applied only to the devices that are descendent
    // of the model where the configuration override was inserted
    for (auto&& params : m_yarpPluginsOverridenParametersList) {
        if (params.gzInstanceId == getGzInstanceId(ecm) &&
            parentEntityScopedNameWhereYARPRobotInterfaceWasInserted.rfind(params.parentEntityScopedNameWhereConfigurationOverrideWasInserted) == 0 &&
            params.overrideType == OverrideType::YARP_ROBOT_INTERFACE &&
            (params.yarpPluginIdentifier == yarpRobotInterfaceName || params.yarpPluginIdentifier == "all")) {
            std::unordered_map<std::string, std::string> consideredOverridenParameters = params.overridenParameters;
            overridenParameters.merge(consideredOverridenParameters);
        }
    }

    return true;
}


} // namespace gzyarp
