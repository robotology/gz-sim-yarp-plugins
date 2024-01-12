#include <gz/common/Event.hh>
#include <mutex>
#include <string>
#include <vector>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

struct LaserData
{
    std::mutex m_mutex;
    std::vector<double> m_data;
    std::string sensorScopedName;
    double simTime;
};

namespace gzyarp
{

class HandlerLaser
{
public:
    static HandlerLaser* getHandler();

    bool setSensor(LaserData* _sensorDataPtr);

    LaserData* getSensor(const std::string& sensorScopedName) const;

    void removeSensor(const std::string& sensorName);

private:
    HandlerLaser();
    static HandlerLaser* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, LaserData*> SensorsMap;
    SensorsMap m_sensorsMap; // map of known sensors
};

} // namespace gzyarp
