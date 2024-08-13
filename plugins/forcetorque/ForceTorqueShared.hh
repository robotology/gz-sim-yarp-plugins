#include <array>
#include <mutex>
#include <string>

namespace gzyarp
{

struct ForceTorqueData
{
    mutable std::mutex m_mutex;
    std::array<double, 6> m_data;
    std::string sensorScopedName;
    double simTime;
};

class IForceTorqueData
{
public:
    virtual void setForceTorqueData(ForceTorqueData*) = 0;

    virtual ~IForceTorqueData() {};
};

} // namespace gzyarp
