#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

namespace gzyarp
{

class DeviceRegistry
{
public:
    static DeviceRegistry* getHandler();

    bool getDevicesAsPolyDriverList(
        const std::string& modelScopedName,
        yarp::dev::PolyDriverList& list,
        std::vector<std::string>& deviceScopedNames /*, const std::string& worldName*/);

    bool setDevice(std::string deviceDatabaseKey, yarp::dev::PolyDriver* device2add);

    yarp::dev::PolyDriver* getDevice(const std::string& deviceDatabaseKey) const;

    void removeDevice(const std::string& deviceDatabaseKey);

    std::vector<std::string> getDevicesKeys() const;

private:
    DeviceRegistry();
    static DeviceRegistry* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, yarp::dev::PolyDriver*> DevicesMap;
    DevicesMap m_devicesMap; // map of known yarp devices
};

} // namespace gzyarp
