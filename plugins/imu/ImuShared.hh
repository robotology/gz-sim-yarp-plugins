#pragma once

#include <array>
#include <mutex>
#include <string>

namespace gzyarp
{

struct ImuData
{
    std::mutex m_mutex;
    std::array<double, 9> m_data;
    std::string sensorScopedName;
    double simTime;
};

class IImuData
{
public:
    virtual void setImuData(ImuData* dataPtr) = 0;

    virtual ~IImuData() {};
};

} // namespace gzyarp
