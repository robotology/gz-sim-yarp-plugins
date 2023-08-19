#include "Handler.hh"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

HandlerForceTorque* HandlerForceTorque::getHandler()
{
    std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle) 
    {
        s_handle = new HandlerForceTorque();
        if (!s_handle)
            yError() << "Error while calling gz-yarp-PluginHandler constructor";
    }
    
    return s_handle;
}

bool HandlerForceTorque::setSensor(ForceTorqueData* _sensorDataPtr)
{
    bool ret = false;
    std::string sensorScopedName = _sensorDataPtr->sensorScopedName;
    SensorsMap::iterator sensor = m_sensorsMap.find(sensorScopedName);

    if (sensor != m_sensorsMap.end()) 
        ret = true;
    else 
    {
        //sensor does not exists. Add to map
        if (!m_sensorsMap.insert(std::pair<std::string, ForceTorqueData*>(sensorScopedName, _sensorDataPtr)).second) 
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


ForceTorqueData* HandlerForceTorque::getSensor(const std::string& sensorScopedName) const
{
    ForceTorqueData* tmp;

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

void HandlerForceTorque::removeSensor(const std::string &sensorName)
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



HandlerForceTorque::HandlerForceTorque() : m_sensorsMap()
{
    m_sensorsMap.clear();
}

HandlerForceTorque* HandlerForceTorque::s_handle = NULL;


std::mutex& HandlerForceTorque::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}




