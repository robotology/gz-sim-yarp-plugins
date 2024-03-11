#include <array>
#include <mutex>
#include <string>

#include <gz/common/Event.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

struct ImuData
{
    std::mutex m_mutex;
    std::array<double, 9> m_data;
    std::string sensorScopedName;
    double simTime;
};

namespace gzyarp
{

class ImuDataSingleton
{
public:
    static ImuDataSingleton* getHandler();

    bool setSensor(ImuData* _sensorDataPtr);

    ImuData* getSensor(const std::string& sensorScopedName) const;

    void removeSensor(const std::string& sensorName);

private:
    ImuDataSingleton();
    static ImuDataSingleton* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, ImuData*> SensorsMap;
    SensorsMap m_sensorsMap; // map of known sensors
};

} // namespace gzyarp
