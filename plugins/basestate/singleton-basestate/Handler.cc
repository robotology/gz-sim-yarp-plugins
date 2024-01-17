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

bool HandlerBaseState::setModel(BaseStateData* _sensorDataPtr)
{
    bool ret = false;
    std::string m_modelScopedName = _sensorDataPtr->m_modelScopedName;
    ModelsMap::iterator sensor = m_modelsMap.find(m_modelScopedName);

    if (sensor != m_modelsMap.end())
        ret = true;
    else
    {
        // sensor does not exists. Add to map
        if (!m_modelsMap
                 .insert(std::pair<std::string, BaseStateData*>(m_modelScopedName, _sensorDataPtr))
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

BaseStateData* HandlerBaseState::getModel(const std::string& m_modelScopedName) const
{
    BaseStateData* tmp;

    ModelsMap::const_iterator sensor = m_modelsMap.find(m_modelScopedName);
    if (sensor != m_modelsMap.end())
    {
        tmp = sensor->second;
    } else
    {
        yError() << "Sensor was not found: " << m_modelScopedName;
        tmp = NULL;
    }
    return tmp;
}

void HandlerBaseState::removeModel(const std::string& sensorName)
{
    ModelsMap::iterator sensor = m_modelsMap.find(sensorName);
    if (sensor != m_modelsMap.end())
    {
        m_modelsMap.erase(sensor);
    } else
    {
        yError() << "Could not remove sensor " << sensorName << ". Sensor was not found";
    }
}

HandlerBaseState::HandlerBaseState()
    : m_modelsMap()
{
    m_modelsMap.clear();
}

HandlerBaseState* HandlerBaseState::s_handle = NULL;

std::mutex& HandlerBaseState::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}

} // namespace gzyarp
