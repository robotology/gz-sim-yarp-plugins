#include "Handler.hh"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

HandlerIMU* HandlerIMU::getHandler()
{
    std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle) 
    {
        s_handle = new HandlerIMU();
        if (!s_handle)
            yError() << "Error while calling gz-yarp-PluginHandler constructor";
    }
    
    return s_handle;
}

bool HandlerIMU::setSensor(IMUData* _sensorDataPtr)
{
    bool ret = false;
    std::string sensorScopedName = _sensorDataPtr->sensorScopedName;
    SensorsMap::iterator sensor = m_sensorsMap.find(sensorScopedName);

    if (sensor != m_sensorsMap.end()) 
        ret = true;
    else 
    {
        //sensor does not exists. Add to map
        if (!m_sensorsMap.insert(std::pair<std::string, IMUData*>(sensorScopedName, _sensorDataPtr)).second) 
        {
            yError() << "Error in Handler while inserting a new sensor pointer!";
            yError() << " The name of the sensor is already present but the pointer does not match with the one already registered!";
            yError() << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error.";
            ret = false;
        } 
        else 
            ret = true;
    }
    return ret;
}

IMUData* HandlerIMU::getSensor(const std::string& sensorScopedName) const
{
    IMUData* tmp;

    SensorsMap::const_iterator sensor = m_sensorsMap.find(sensorScopedName);
    if (sensor != m_sensorsMap.end()) 
    {
        tmp = sensor->second;
    } 
    else 
    {
        yError() << "Sensor was not found: " << sensorScopedName;
        tmp = NULL;
    }
    return tmp;
}

void HandlerIMU::removeSensor(const std::string &sensorName)
{
    SensorsMap::iterator sensor = m_sensorsMap.find(sensorName);
    if (sensor != m_sensorsMap.end()) 
    {
        m_sensorsMap.erase(sensor);
    } 
    else 
    {
        yError() << "Could not remove sensor " << sensorName << ". Sensor was not found";
    }
}

HandlerIMU::HandlerIMU() : m_sensorsMap()
{
    m_sensorsMap.clear();
}

HandlerIMU* HandlerIMU::s_handle = NULL;


std::mutex& HandlerIMU::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}




