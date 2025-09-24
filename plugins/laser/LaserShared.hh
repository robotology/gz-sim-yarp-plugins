#pragma once

#include <mutex>
#include <string>
#include <vector>

namespace gzyarp
{

struct LaserData
{
    std::vector<double> m_data;
    double              m_simTime;
};

class ILaserData
{
public:
    virtual void updateLaserMeasurements(const LaserData& gzdata) = 0;

    virtual ~ILaserData() {};
};

} // namespace gzyarp
