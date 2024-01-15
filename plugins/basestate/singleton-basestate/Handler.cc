#include "Handler.hh"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gzyarp
{

HandlerBaseState* HandlerBaseState::getHandler()
{
    std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle)
    {
        s_handle = new HandlerBaseState();
        if (!s_handle)
            yError() << "Error while calling gzyarp::HandlerBaseState constructor";
    }

    return s_handle;
}

bool HandlerBaseState::setSensor(BaseStateData* _sensorDataPtr)
{
    bool ret = false;
    std::string sensorScopedName = _sensorDataPtr->sensorScopedName;
    SensorsMap::iterator sensor = m_sensorsMap.find(sensorScopedName);

    if (sensor != m_sensorsMap.end())
        ret = true;
    else
    {
        // sensor does not exists. Add to map
        if (!m_sensorsMap
                 .insert(std::pair<std::string, BaseStateData*>(sensorScopedName, _sensorDataPtr))
                 .second)
        {
            yError() << "Error in gzyarp::HandlerBaseState while inserting a new sensor pointer!";
            yError() << " The name of the sensor is already present but the pointer does not match "
                        "with the one already registered!";
            yError() << " This should not happen, as the scoped name should be unique in Gazebo. "
                        "Fatal error.";
            ret = false;
        } else
            ret = true;
    }
    return ret;
}

BaseStateData* HandlerBaseState::getSensor(const std::string& sensorScopedName) const
{
    BaseStateData* tmp;

    SensorsMap::const_iterator sensor = m_sensorsMap.find(sensorScopedName);
    if (sensor != m_sensorsMap.end())
    {
        tmp = sensor->second;
    } else
    {
        yError() << "Sensor was not found: " << sensorScopedName;
        tmp = NULL;
    }
    return tmp;
}

void HandlerBaseState::removeSensor(const std::string& sensorName)
{
    SensorsMap::iterator sensor = m_sensorsMap.find(sensorName);
    if (sensor != m_sensorsMap.end())
    {
        m_sensorsMap.erase(sensor);
    } else
    {
        yError() << "Could not remove sensor " << sensorName << ". Sensor was not found";
    }
}

HandlerBaseState::HandlerBaseState()
    : m_sensorsMap()
{
    m_sensorsMap.clear();
}

HandlerBaseState* HandlerBaseState::s_handle = NULL;

std::mutex& HandlerBaseState::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}

} // namespace gzyarp
