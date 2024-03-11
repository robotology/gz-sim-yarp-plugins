#include <ImuDataSingleton.hh>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gzyarp
{

ImuDataSingleton* ImuDataSingleton::getHandler()
{
    std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle)
    {
        s_handle = new ImuDataSingleton();
        if (!s_handle)
            yError() << "Error while calling gzyarp::HandlerIMU constructor";
    }

    return s_handle;
}

bool ImuDataSingleton::setSensor(ImuData* _sensorDataPtr)
{
    bool ret = false;
    std::string sensorScopedName = _sensorDataPtr->sensorScopedName;
    SensorsMap::iterator sensor = m_sensorsMap.find(sensorScopedName);

    if (sensor != m_sensorsMap.end())
        ret = true;
    else
    {
        // sensor does not exists. Add to map
        if (!m_sensorsMap.insert(std::pair<std::string, ImuData*>(sensorScopedName, _sensorDataPtr))
                 .second)
        {
            yError() << "Error in gzyarp::HandlerIMU while inserting a new sensor pointer!";
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

ImuData* ImuDataSingleton::getSensor(const std::string& sensorScopedName) const
{
    ImuData* tmp;

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

void ImuDataSingleton::removeSensor(const std::string& sensorName)
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

ImuDataSingleton::ImuDataSingleton()
    : m_sensorsMap()
{
    m_sensorsMap.clear();
}

ImuDataSingleton* ImuDataSingleton::s_handle = NULL;

std::mutex& ImuDataSingleton::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}

} // namespace gzyarp
