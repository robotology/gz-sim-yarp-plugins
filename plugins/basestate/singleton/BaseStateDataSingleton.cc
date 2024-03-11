#include <BaseStateDataSingleton.hh>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gzyarp
{

BaseStateDataSingleton* BaseStateDataSingleton::getBaseStateDataHandler()
{
    {
        std::lock_guard<std::mutex> lock(mutex());
        if (!s_handle)
        {
            s_handle = new BaseStateDataSingleton();
            if (!s_handle)
            {
                yError() << "Error while calling gzyarp::HandlerBaseState constructor";
            }
        }
    }
    return s_handle;
}

bool BaseStateDataSingleton::setBaseStateData(BaseStateData* _baseStateDataPtr)
{
    bool ret = false;
    BaseLinksMap::iterator baseLink = m_baseLinksMap.find(_baseStateDataPtr->baseLinkScopedName);

    if (baseLink != m_baseLinksMap.end())
    {
        ret = true;
    } else
    {
        // baseLink does not exists. Add to map
        if (!m_baseLinksMap
                 .insert(
                     std::pair<std::string, BaseStateData*>(_baseStateDataPtr->baseLinkScopedName,
                                                            _baseStateDataPtr))
                 .second)
        {
            yError() << "Error in gzyarp::HandlerBaseState while inserting a new base link "
                        "pointer!";
            yError() << "The name of the base link is already present but the pointer does not "
                        "match "
                        "with the one already registered!";
            yError() << "This should not happen, as the scoped name should be unique in Gazebo. "
                        "Fatal error.";
            ret = false;
        } else
        {
            ret = true;
        }
    }
    return ret;
}

BaseStateData*
BaseStateDataSingleton::getBaseStateData(const std::string& _baseLinkScopedName) const
{
    BaseStateData* dataPtr;

    BaseLinksMap::const_iterator baseLink = m_baseLinksMap.find(_baseLinkScopedName);
    if (baseLink != m_baseLinksMap.end())
    {
        dataPtr = baseLink->second;
    } else
    {
        yError() << "Base link was not found: " << _baseLinkScopedName;
        dataPtr = nullptr;
    }
    return dataPtr;
}

void BaseStateDataSingleton::removeBaseLink(const std::string& _baseLinkScopedName)
{
    BaseLinksMap::iterator baseLink = m_baseLinksMap.find(_baseLinkScopedName);
    if (baseLink != m_baseLinksMap.end())
    {
        m_baseLinksMap.erase(baseLink);
    } else
    {
        yError() << "Could not remove base link " << _baseLinkScopedName
                 << ". Base link was not found";
    }
}

BaseStateDataSingleton::BaseStateDataSingleton()
    : m_baseLinksMap()
{
    m_baseLinksMap.clear();
}

BaseStateDataSingleton* BaseStateDataSingleton::s_handle = NULL;

std::mutex& BaseStateDataSingleton::mutex()
{
    static std::mutex s_mutex;
    return s_mutex;
}

} // namespace gzyarp
