#include <array>
#include <gz/common/Event.hh>
#include <mutex>
#include <string>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

struct BaseStateData
{
    std::mutex m_mutex;
    std::string m_modelScopedName;
    bool m_dataAvailable;
    std::array<double, 18> m_data;
    yarp::os::Stamp m_simTimestamp;
};

namespace gzyarp
{

class HandlerBaseState
{

public:
    static HandlerBaseState* getHandler();

    bool setModel(BaseStateData* _modelDataPtr);

    BaseStateData* getModel(const std::string& m_modelScopedName) const;

    void removeModel(const std::string& modelName);

private:
    HandlerBaseState();
    static HandlerBaseState* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, BaseStateData*> ModelsMap;
    ModelsMap m_modelsMap; // map of known sensors
};

} // namespace gzyarp
