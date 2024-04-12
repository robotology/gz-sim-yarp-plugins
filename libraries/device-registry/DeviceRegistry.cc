#include <DeviceRegistry.hh>

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gzyarp
{

DeviceRegistry* DeviceRegistry::getHandler()
{
    std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle)
    {
        s_handle = new DeviceRegistry();
        if (!s_handle)
            yError() << "Error while calling gzyarp::Handler constructor";
    }

    return s_handle;
}

bool DeviceRegistry::getDevicesAsPolyDriverList(
    const std::string& modelScopedName,
    yarp::dev::PolyDriverList& list,
    std::vector<std::string>& deviceScopedNames /*, const std::string& worldName*/)
{
    deviceScopedNames.resize(0);

    list = yarp::dev::PolyDriverList();

    // This map contains only the yarpDeviceName that we actually added
    // to the returned yarp::dev::PolyDriverList
    std::unordered_map<std::string, std::string> inserted_yarpDeviceName2deviceDatabaseKey;

    for (auto&& devicesMapElem : m_devicesMap)
    {
        std::string deviceDatabaseKey = devicesMapElem.first;
        std::string yarpDeviceName;

        // If the deviceDatabaseKey starts with the modelScopedName (device spawned by model
        // plugins), then it is eligible for insertion in the returned list
        if ((deviceDatabaseKey.rfind(modelScopedName, 0)
             == 0) /*|| (deviceDatabaseKey.rfind(worldName + "/" + modelScopedName, 0) == 0)*/)
        {
            // Extract yarpDeviceName from deviceDatabaseKey
            yarpDeviceName = deviceDatabaseKey.substr(deviceDatabaseKey.find_last_of("/") + 1);

            // Check if a device with the same yarpDeviceName was already inserted
            auto got = inserted_yarpDeviceName2deviceDatabaseKey.find(yarpDeviceName);

            // If not found, insert and continue
            if (got == inserted_yarpDeviceName2deviceDatabaseKey.end())
            {
                // If no name collision is found, insert and continue
                inserted_yarpDeviceName2deviceDatabaseKey.insert(
                    {yarpDeviceName, deviceDatabaseKey});
                list.push(devicesMapElem.second, yarpDeviceName.c_str());
                deviceScopedNames.push_back(deviceDatabaseKey);
            } else
            {
                // If a name collision is found, print a clear error and return
                yError() << "gzyarp::Handler robotinterface getDevicesAsPolyDriverList error: ";
                yError() << "two YARP devices with yarpDeviceName " << yarpDeviceName
                         << " found in model " << modelScopedName;
                yError() << "First instance: " << got->second;
                yError() << "Second instance: " << deviceDatabaseKey;
                yError() << "Please eliminate or rename one of the two instances. ";
                list = yarp::dev::PolyDriverList();
                deviceScopedNames.resize(0);
                return false;
            }
        }
    }
    return true;
}

bool DeviceRegistry::setDevice(std::string deviceDatabaseKey, yarp::dev::PolyDriver* device2add)
{
    bool ret = false;
    DevicesMap::iterator device = m_devicesMap.find(deviceDatabaseKey);
    if (device != m_devicesMap.end())
    {
        if (device->second == device2add)
            ret = true;
        else
        {
            yError() << " Error in gzyarp::Handler while inserting a new yarp device pointer!";
            yError() << " The name of the device is already present but the pointer does not match "
                        "with the one already registered!";
            yError() << " This should not happen, check the names are correct in your config file. "
                        "Fatal error.";
        }
    } else
    {
        // device does not exists. Add to map
        if (!m_devicesMap
                 .insert(
                     std::pair<std::string, yarp::dev::PolyDriver*>(deviceDatabaseKey, device2add))
                 .second)
        {
            yError() << " Error in gzyarp::Handler while inserting a new device pointer!";
            ret = false;
        } else
            ret = true;
    }
    return ret;
}

yarp::dev::PolyDriver* DeviceRegistry::getDevice(const std::string& deviceDatabaseKey) const
{
    yarp::dev::PolyDriver* tmp = NULL;

    DevicesMap::const_iterator device = m_devicesMap.find(deviceDatabaseKey);
    if (device != m_devicesMap.end())
        tmp = device->second;
    else
        tmp = NULL;

    return tmp;
}

void DeviceRegistry::removeDevice(const std::string& deviceDatabaseKey)
{
    DevicesMap::iterator device = m_devicesMap.find(deviceDatabaseKey);
    if (device != m_devicesMap.end())
    {
        device->second->close();
        m_devicesMap.erase(device);
    } else
    {
        yError() << "Could not remove device " << deviceDatabaseKey << ". Device was not found";
    }
    return;
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

} // namespace gzyarp
