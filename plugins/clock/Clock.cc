#include <DeviceRegistry.hh>

#include <chrono>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

#include <thread>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>

using namespace gz;
using namespace sim;
using namespace systems;

using yarp::os::Bottle;
using yarp::os::BufferedPort;

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
            gzyarp::PluginConfigureHelper configureHelper(_ecm);

            // To avoid deadlock during initialization if YARP_CLOCK is set,
            // if the YARP network is not initialized we always initialize
            // it with system clock, and then we switch back to the default clock later
            bool networkIsNotInitialized = !yarp::os::NetworkBase::isNetworkInitialized();

            if (networkIsNotInitialized)
            {
                m_network = yarp::os::Network(yarp::os::YARP_CLOCK_SYSTEM);
                m_resetYARPClockAfterPortCreation = true;
            } else
            {
                m_network = yarp::os::Network();
                m_resetYARPClockAfterPortCreation = false;
            }

            m_initialized = true;
            if (!m_clockPort.open(m_portName))
            {
                yError() << "Failed to open port" << m_portName;
                return;
            }

            configureHelper.setConfigureIsSuccessful(true);
            yInfo() << "gz-sim-yarp-clock-system plugin initialized.";
        }
    }

    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        if (_info.paused)
        {
            return;
        }

        // As the port is now created and contains streams data,
        // if necessary reset the YARP clock to YARP_CLOCK_DEFAULT
        // Unfortunately, the yarpClockInit blocks on the port until it
        // receives data, so we need to launch it in a different thread
        if (m_resetYARPClockAfterPortCreation)
        {
            yError() << "Resetting YARP clock to default";
            auto resetYARPNetworkClockLambda
                = []() { yarp::os::NetworkBase::yarpClockInit(yarp::os::YARP_CLOCK_DEFAULT); };
            std::thread resetYARPNetworkClockThread(resetYARPNetworkClockLambda);
            resetYARPNetworkClockThread.detach();
            m_resetYARPClockAfterPortCreation = false;
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
    // True if the YARP network needs to be reset to
    // YARP_CLOCK_DEFAULT after the port has been created
    bool m_resetYARPClockAfterPortCreation;
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
