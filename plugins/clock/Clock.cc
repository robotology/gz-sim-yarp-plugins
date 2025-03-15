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
        std::cerr << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~> Clock: Destructor " << std::endl;
        m_clockPort.close();
    }

    // This helper function is used to publish the current time
    void publishTime(const std::chrono::steady_clock::duration& timeToPublish)
    {
        m_lastPublishedTime = timeToPublish;
        std::chrono::seconds seconds
            = std::chrono::duration_cast<std::chrono::seconds>(timeToPublish);
        std::chrono::nanoseconds nanoseconds = timeToPublish - seconds;

        Bottle& b = m_clockPort.prepare();
        b.clear();
        b.addInt32(seconds.count());
        b.addInt32(nanoseconds.count());
        m_clockPort.write();
    }

    // This function is used by a thread that publishes increasing time values on the clock
    // on the event of a reset, to unblock any thread that is waiting for the simulated clock
    void threadBodyPublishIncreasingTimestampsOnReset()
    {
        auto publishingPeriod = std::chrono::milliseconds(10);
        auto timeWhenToPublish = std::chrono::steady_clock::now();
        while (m_gazeboResetOngoing) {
            // We publish time on top of the simulated once
            auto timeToPublish = m_lastPublishedTime + publishingPeriod;
            publishTime(timeToPublish);

            // Schedule next execution time
            timeWhenToPublish += publishingPeriod;
            std::this_thread::sleep_until(timeWhenToPublish);
        }
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
                m_network = std::make_unique<yarp::os::Network>(yarp::os::YARP_CLOCK_SYSTEM);
                m_resetYARPClockAfterPortCreation = true;
            } else
            {
                m_network = std::make_unique<yarp::os::Network>();
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
        std::cerr << "==================> PostUpdate: running with simTime" << std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count() << " and realtime " << std::chrono::duration_cast<std::chrono::microseconds>(_info.realTime).count()  << std::endl;
        if (m_gazeboResetOngoing)
        {
            // The reset phase has ended, let's stop the thread that publishes increasing time values
            m_gazeboResetOngoing = false;
            m_threadPublishIncreasingTimestampsOnReset.join();
        }

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
            auto resetYARPNetworkClockLambda
                = []() { yarp::os::NetworkBase::yarpClockInit(yarp::os::YARP_CLOCK_DEFAULT); };
            std::thread resetYARPNetworkClockThread(resetYARPNetworkClockLambda);
            resetYARPNetworkClockThread.detach();
            m_resetYARPClockAfterPortCreation = false;
        }

        publishTime(_info.simTime);
    }

    void Reset(const UpdateInfo& _info, EntityComponentManager& /*_ecm*/) override
    {
        std::cerr << "~~~~~~~~~~~~~~~~~> Reset: running with simTime" << std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count() << " and realtime " << std::chrono::duration_cast<std::chrono::microseconds>(_info.realTime).count()  << std::endl;

        // At a reset, we setup a separate thread that publish on the port increasing values,
        // so that any thread that is waiting to exist based on clock data will be unblocked,
        // solving issues like https://github.com/robotology/gz-sim-yarp-plugins/issues/252

        // At the first PostUpdate after the reset, the thread will be stopped and the clock
        // will contain the correct time (tipically 0.0), and yarp::os::NetworkClock handles the reset correctly, see
        // https://github.com/robotology/yarp/blob/v3.11.2/src/libYARP_os/src/yarp/os/NetworkClock.cpp#L112-L116

        // First of all we set the flag that indicates that a reset is ongoing
        m_gazeboResetOngoing = true;

        // Then we start the thread that publishes increasing time values
        m_threadPublishIncreasingTimestampsOnReset = std::thread(&Clock::threadBodyPublishIncreasingTimestampsOnReset, this);

    }

private:
    bool m_initialized;
    // True if the YARP network needs to be reset to
    // YARP_CLOCK_DEFAULT after the port has been created
    bool m_resetYARPClockAfterPortCreation;
    std::unique_ptr<yarp::os::Network> m_network = nullptr;
    std::string m_portName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_clockPort;
    // Last time published by the clock
    std::chrono::steady_clock::duration m_lastPublishedTime{0};

    // True between a call to Reset and the first subsequent call to PostUpdate
    std::atomic<bool> m_gazeboResetOngoing{false};

    // Thread that publishes increasing time values on the clock on reset
    std::thread m_threadPublishIncreasingTimestampsOnReset;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::Clock,
              gz::sim::System,
              gzyarp::Clock::ISystemConfigure,
              gzyarp::Clock::ISystemPostUpdate,
              gzyarp::Clock::ISystemReset)
