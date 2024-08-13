#pragma once

#include <mutex>
#include <string>
#include <vector>

namespace gzyarp
{

struct LaserData
{
    std::mutex m_mutex;
    std::vector<double> m_data;
    std::string sensorScopedName;
    double simTime;
};

class ILaserData
{
public:
    virtual void setLaserData(LaserData* dataPtr) = 0;

    virtual ~ILaserData() {};
};

} // namespace gzyarp
