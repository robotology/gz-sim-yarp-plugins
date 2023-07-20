#include <mutex>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <gz/common/Event.hh>

struct ForceTorqueData
{
  std::mutex m_mutex;
  std::array<double, 6> m_data;
  std::string sensorScopedName;
  double simTime;
};


class HandlerForceTorque
{   
    public:
        static HandlerForceTorque* getHandler();

        bool setSensor(ForceTorqueData* _sensorDataPtr);

        ForceTorqueData* getSensor(const std::string& sensorScopedName) const;

        void removeSensor(const std::string& sensorName);
        
    private:
        HandlerForceTorque();
        static HandlerForceTorque* s_handle;
        static std::mutex& mutex();
        typedef std::map<std::string, ForceTorqueData*> SensorsMap;
        SensorsMap m_sensorsMap;    // map of known sensors

};


