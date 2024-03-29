#include <ControlBoardDataSingleton.hh>

#include <ControlBoardData.hh>

#include <mutex>
#include <string>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gzyarp
{

ControlBoardDataSingleton* ControlBoardDataSingleton::getControlBoardHandler()
{
    std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle)
    {
        s_handle = new ControlBoardDataSingleton();
        if (!s_handle)
        {
            yError() << "Error while calling gzyarp::ControlBoardDataSingleton constructor";
        }
    }
    return s_handle;
}

bool ControlBoardDataSingleton::setControlBoardData(ControlBoardData* _controlBoardPtr)
{
    bool ret = false;
    ControlBoardMap::iterator controlBoard
        = m_controlBoardMap.find(_controlBoardPtr->controlBoardId);

    if (controlBoard != m_controlBoardMap.end())
    {
        yDebug() << "Control board scoped name: " << _controlBoardPtr->controlBoardId
                 << " already present.\n";
        ret = true;
    } else
    {
        // controlBoard does not exists. Add to map
        if (!m_controlBoardMap
                 .insert(std::pair<std::string, ControlBoardData*>(_controlBoardPtr->controlBoardId,
                                                                   _controlBoardPtr))
                 .second)
        {
            yError() << "Error in gzyarp::ControlBoardDataSingleton while inserting a new control "
                        "board pointer!";
            yError() << "The name of the control board is already present but the pointer does not "
                        "match "
                        "with the one already registered!";
            yError() << "This should not happen, as the scoped name should be unique in Gazebo. "
                        "Fatal error.";
            ret = false;
        } else
        {
            ret = true;
            yDebug() << "Added control board: " << _controlBoardPtr->controlBoardId;
        }
    }
    return ret;
}

ControlBoardData*
ControlBoardDataSingleton::getControlBoardData(const std::string& _controlBoardScopedName) const
{
    ControlBoardData* dataPtr;
    ControlBoardMap::const_iterator controlBoard = m_controlBoardMap.find(_controlBoardScopedName);
    if (controlBoard != m_controlBoardMap.end())
    {
        dataPtr = controlBoard->second;
    } else
    {
        dataPtr = nullptr;
    }
    return dataPtr;
}

void ControlBoardDataSingleton::removeControlBoard(const std::string& _controlBoardScopedName)
{
    ControlBoardMap::iterator controlBoard = m_controlBoardMap.find(_controlBoardScopedName);
    if (controlBoard != m_controlBoardMap.end())
    {
        m_controlBoardMap.erase(controlBoard);
    } else
    {
        yError() << "Control board was not found: " << _controlBoardScopedName;
    }
}

std::vector<std::string> ControlBoardDataSingleton::getControlBoardKeys() const
{
    std::vector<std::string> keys;
    for (const auto& controlBoard : m_controlBoardMap)
    {
        keys.push_back(controlBoard.first);
    }
    return keys;
}

// Private methods

ControlBoardDataSingleton::ControlBoardDataSingleton()
    : m_controlBoardMap()
{
    m_controlBoardMap.clear();
}

ControlBoardDataSingleton* ControlBoardDataSingleton::s_handle = nullptr;

std::mutex& ControlBoardDataSingleton::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}

} // namespace gzyarp
