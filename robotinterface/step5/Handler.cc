#include "Handler.hh"
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



Handler* Handler::getHandler()
{
    //std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle) {
        s_handle = new Handler();
        if (!s_handle)
            yError() << "Error while calling GazeboYarpPluginHandler constructor";
    }
    

    return s_handle;
}

bool Handler::setSensor(EntityComponentManager &_ecm, gz::sim::Sensor _sensorPtr)
{
    bool ret = false;
    std::string sensorScopedName = scopedName(_sensorPtr.Entity(), _ecm);
    SensorsMap::iterator sensor = m_sensorsMap.find(sensorScopedName);

    if (sensor != m_sensorsMap.end()) {
        ret = true;
    } else {
        //sensor does not exists. Add to map
        //ReferenceCountingSensor countedSensor(_sensorPtr);
        if (!m_sensorsMap.insert(std::pair<std::string, gz::sim::Sensor>(sensorScopedName, _sensorPtr)).second) {
            yError() << "Error in Handler while inserting a new sensor pointer!";
            yError() << " The name of the sensor is already present but the pointer does not match with the one already registered!";
            yError() << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error.";
            ret = false;
        } else {
            ret = true;
        }
    }
    return ret;
}

gz::sim::Sensor Handler::getSensor(const std::string& sensorScopedName) const
{
    gz::sim::Sensor tmp;

    SensorsMap::const_iterator sensor = m_sensorsMap.find(sensorScopedName);
    if (sensor != m_sensorsMap.end()) {
        tmp = Sensor(sensor->second.Entity());
    } else {
        yError() << "Sensor was not found: " << sensorScopedName;
        // tmp = NULL;
    }
    return tmp;
}

bool Handler::setDevice(std::string deviceDatabaseKey, yarp::dev::PolyDriver* device2add)
{
    bool ret = false;
    DevicesMap::iterator device = m_devicesMap.find(deviceDatabaseKey);
    if (device != m_devicesMap.end()) {
        //device already exists. Increment reference counting
        if(device->second == device2add)
        {
            //device->second.incrementCount();
            ret = true;
        }
        else
        {
            yError() << " Error in GazeboYarpPlugins::Handler while inserting a new yarp device pointer!";
            yError() << " The name of the device is already present but the pointer does not match with the one already registered!";
            yError() << " This should not happen, check the names are correct in your config file. Fatal error.";
        }
    } 
    else {
        //device does not exists. Add to map
        //ReferenceCountingDevice countedDevice(device2add);
        if (!m_devicesMap.insert(std::pair<std::string, yarp::dev::PolyDriver*>(deviceDatabaseKey, device2add)).second) {
            yError() << " Error in GazeboYarpPlugins::Handler while inserting a new device pointer!";
            ret = false;
        } else {
            ret = true;
        }
    }
    return ret;
}

yarp::dev::PolyDriver* Handler::getDevice(const std::string& deviceDatabaseKey) const
{
    yarp::dev::PolyDriver* tmp = NULL;

    DevicesMap::const_iterator device = m_devicesMap.find(deviceDatabaseKey);
    if (device != m_devicesMap.end()) {
        tmp = device->second;
    } else {
        tmp = NULL;
    }
    return tmp;
}


Handler::Handler() : m_sensorsMap(), m_devicesMap()
{
    m_sensorsMap.clear();
    m_devicesMap.clear();
}

Handler* Handler::s_handle = NULL;


std::mutex& Handler::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}




