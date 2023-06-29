#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Sensor.hh>
#include <sdf/Element.hh>
#include <mutex>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <gz/common/Event.hh>
#include <unordered_map> 


using namespace gz;
using namespace sim;
using namespace systems;


class Handler
{   
    public:
    static Handler* getHandler();

    bool setSensor(EntityComponentManager &_ecm, gz::sim::Sensor _sensorPtr);

    gz::sim::Sensor getSensor(const std::string& sensorScopedName) const;


    bool setDevice(std::string deviceDatabaseKey, yarp::dev::PolyDriver* device2add);
    

    yarp::dev::PolyDriver* getDevice(const std::string& deviceDatabaseKey) const;

    private:
        Handler();
        
        static Handler* s_handle;


        static std::mutex& mutex();

        typedef std::map<std::string, gz::sim::Sensor> SensorsMap;
        typedef std::map<std::string, yarp::dev::PolyDriver*> DevicesMap;
        SensorsMap m_sensorsMap;    // map of known sensors
        DevicesMap m_devicesMap;    // map of known yarp devices

};


