#pragma once

#include <array>
#include <mutex>
#include <string>
#include <yarp/os/Stamp.h>

namespace gzyarp
{

struct BaseStateData
{
    std::mutex mutex;
    std::string baseLinkScopedName;
    bool dataAvailable;
    std::array<double, 18> data;
    yarp::os::Stamp simTimestamp;
};

class IBaseStateData
{
public:
    virtual void setBaseStateData(BaseStateData*) = 0;

    virtual ~IBaseStateData() {};
};

} // namespace gzyarp
