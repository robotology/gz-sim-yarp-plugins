#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

using namespace gz;
using namespace sim;
using namespace systems;

using yarp::os::Log;

namespace gzyarp
{

class Clock : public System, public ISystemPostUpdate
{
public:
    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        std::cout << "SimTime: " << _info.simTime.count() << std::endl;
    }

private:
    yarp::os::Network* m_network;
    std::string m_portName;
    yarp::os::BufferedPort<yarp::os::Bottle>* m_clockPort;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::Clock, gz::sim::System, gzyarp::Clock::ISystemPostUpdate)
