#include <array>
#include <mutex>
#include <string>

#include <gz/common/Event.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

struct ForceTorqueData
{
    std::mutex m_mutex;
    std::array<double, 6> m_data;
    std::string sensorScopedName;
    double simTime;
};

namespace gzyarp
{

class ForceTorqueDataSingleton
{
public:
    static ForceTorqueDataSingleton* getHandler();

    bool setSensor(ForceTorqueData* _sensorDataPtr);

    ForceTorqueData* getSensor(const std::string& sensorScopedName) const;

    void removeSensor(const std::string& sensorName);

private:
    ForceTorqueDataSingleton();
    static ForceTorqueDataSingleton* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, ForceTorqueData*> SensorsMap;
    SensorsMap m_sensorsMap; // map of known sensors
};

} // namespace gzyarp
