#include "../include/ControlBoardDriver.hh"

#include "../include/ControlBoardDataSingleton.hh"

namespace yarp
{
namespace dev
{
namespace gzyarp
{

// DeviceDriver
bool ControlBoardDriver::open(yarp::os::Searchable& config)
{
    auto m_controlBoardScopedName = config.find(YarpControlBoardScopedName).asString();

    m_controlBoardData
        = ::gzyarp::ControlBoardDataSingleton::getControlBoardHandler()->getControlBoardData(
            m_controlBoardScopedName);
}

bool ControlBoardDriver::close()
{
    return true;
}

} // namespace gzyarp
} // namespace dev
} // namespace yarp
