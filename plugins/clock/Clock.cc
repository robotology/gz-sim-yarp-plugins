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

using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::Log;
using yarp::os::Network;

namespace gzyarp
{

class Clock : public System, public ISystemPostUpdate
{
public:
    Clock()
    {
        m_portName = "/clock";
        m_clockPort.open(m_portName);
    }

    ~Clock()
    {
        m_clockPort.close();
    }

    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        Bottle& b = m_clockPort.prepare();
        b.clear();
        b.addInt64(_info.simTime.count());
        m_clockPort.write();
    }

private:
    yarp::os::Network m_network;
    std::string m_portName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_clockPort;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::Clock, gz::sim::System, gzyarp::Clock::ISystemPostUpdate)
