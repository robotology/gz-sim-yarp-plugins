#include <chrono>
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

class Clock : public System, public ISystemConfigure, public ISystemPostUpdate, public ISystemReset
{
public:
    Clock()
        : m_portName("/clock")
        , m_initialized(false)
    {
    }

    ~Clock()
    {
        m_clockPort.close();
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        if (!m_initialized)
        {
            m_initialized = true;
            if (!m_clockPort.open(m_portName))
            {
                yError() << "Failed to open port" << m_portName;
                return;
            }
            yInfo() << "gz-sim-yarp-clock-system plugin initialized.";
        }
    }

    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        if (_info.paused)
        {
            return;
        }

        auto currentTime = _info.simTime;
        std::chrono::seconds seconds
            = std::chrono::duration_cast<std::chrono::seconds>(currentTime);
        std::chrono::nanoseconds nanoseconds = currentTime - seconds;

        Bottle& b = m_clockPort.prepare();
        b.clear();
        b.addInt32(seconds.count());
        b.addInt32(nanoseconds.count());
        m_clockPort.write();
    }

    void Reset(const UpdateInfo& _info, EntityComponentManager& _ecm) override
    {
        yInfo() << "gz-sim-yarp-clock-system plugin Reset invoked.";
    }

private:
    bool m_initialized;
    yarp::os::Network m_network;
    std::string m_portName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_clockPort;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::Clock,
              gz::sim::System,
              gzyarp::Clock::ISystemConfigure,
              gzyarp::Clock::ISystemPostUpdate,
              gzyarp::Clock::ISystemReset)
