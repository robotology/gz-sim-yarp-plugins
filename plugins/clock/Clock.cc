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

using std::chrono::nanoseconds;
using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::Log;
using yarp::os::Network;

namespace gzyarp
{

class Clock : public System, public ISystemConfigure, public ISystemPostUpdate
{
public:
    Clock()
        : m_portName("/clock")
        , m_initialized(false)
        , m_lastTimestampSent(0)
    {
        std::cout << "===========> constructor" << std::endl;
    }

    ~Clock()
    {
        std::cout << "===========> destructor" << std::endl;
        m_clockPort.close();
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        std::cout << "===========> configure" << std::endl;

        if (!m_initialized)
        {
            m_initialized = true;
            if (!m_clockPort.open(m_portName))
            {
                yError() << "Failed to open port" << m_portName;
                return;
            }
            yInfo() << "Clock plugin initialized";
        }
    }

    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        if (_info.simTime.count() == m_lastTimestampSent.count())
        {
            yDebug() << "Skipping clock update, same timestamp as last update";
            return;
        }

        Bottle& b = m_clockPort.prepare();
        b.clear();
        b.addInt64(_info.simTime.count());
        m_clockPort.write();

        m_lastTimestampSent = nanoseconds(_info.simTime.count());
    }

private:
    bool m_initialized;
    yarp::os::Network m_network;
    std::string m_portName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_clockPort;
    nanoseconds m_lastTimestampSent;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::Clock,
              gz::sim::System,
              gzyarp::Clock::ISystemConfigure,
              gzyarp::Clock::ISystemPostUpdate)
