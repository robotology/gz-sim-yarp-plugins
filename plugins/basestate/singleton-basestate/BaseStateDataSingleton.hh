#include <array>
#include <gz/common/Event.hh>
#include <mutex>
#include <string>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Stamp.h>

struct BaseStateData
{
    std::mutex mutex;
    std::string baseLinkScopedName;
    bool dataAvailable;
    std::array<double, 18> data;
    yarp::os::Stamp simTimestamp;
};

namespace gzyarp
{

class BaseStateDataSingleton
{

public:
    static BaseStateDataSingleton* getBaseStateDataHandler();

    bool setBaseStateData(BaseStateData* _baseStateDataPtr);

    BaseStateData* getBaseStateData(const std::string& _baseLinkScopedName) const;

    void removeBaseLink(const std::string& _baseLinkScopedName);

private:
    BaseStateDataSingleton();
    static BaseStateDataSingleton* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, BaseStateData*> BaseLinksMap;
    BaseLinksMap m_baseLinksMap; // map of base links with their scoped name as key
};

} // namespace gzyarp
