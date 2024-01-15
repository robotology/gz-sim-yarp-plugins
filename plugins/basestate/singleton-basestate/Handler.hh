#include <array>
#include <gz/common/Event.hh>
#include <mutex>
#include <string>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

struct BaseStateData
{
    std::mutex m_mutex;
    std::array<double, 6> m_data;
    std::string sensorScopedName;
    double simTime;
};

namespace gzyarp
{

class HandlerBaseState
{

public:
    static HandlerBaseState* getHandler();

    bool setSensor(BaseStateData* _sensorDataPtr);

    BaseStateData* getSensor(const std::string& sensorScopedName) const;

    void removeSensor(const std::string& sensorName);

private:
    HandlerBaseState();
    static HandlerBaseState* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, BaseStateData*> SensorsMap;
    SensorsMap m_sensorsMap; // map of known sensors
};

} // namespace gzyarp
