#include <mutex>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <gz/common/Event.hh>

struct IMUData
{
  std::mutex m_mutex;
  std::array<double, 9> m_data;
  std::string sensorScopedName;
  double simTime;
};


class HandlerIMU
{   
    public:
        static HandlerIMU* getHandler();

        bool setSensor(IMUData* _sensorDataPtr);

        IMUData* getSensor(const std::string& sensorScopedName) const;

        void removeSensor(const std::string& sensorName);
 
    private:
        HandlerIMU();
        static HandlerIMU* s_handle;
        static std::mutex& mutex();
        typedef std::map<std::string, IMUData*> SensorsMap;
        SensorsMap m_sensorsMap;    // map of known sensors

};


