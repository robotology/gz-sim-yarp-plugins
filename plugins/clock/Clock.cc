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

class Clock : public System, public ISystemConfigure, public ISystemPostUpdate

{
public:
    Clock()
        : m_portName("/clock")
        , m_initialized(false)
    {
    }

    ~Clock()
    {
        if (m_initialized)
        {
            m_clockPluginDestructionOngoing = true;

            std::thread threadPublishIncreasingTimestampsOnDestruction = std::thread(&Clock::threadBodyPublishIncreasingTimestampsOnDestruction, this);

            DeviceRegistry::getHandler()->removeClockPlugin(*ecm, m_clockPluginID);

            m_clockPluginDestructionOngoing = false;

            threadPublishIncreasingTimestampsOnDestruction.join();

            m_clockPort.close();

            m_initialized = false;
        }
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
    // on the event the clock plugin is destroyed, to unblock any thread that is waiting for the simulated clock
    void threadBodyPublishIncreasingTimestampsOnDestruction()
    {
        auto publishingPeriod = std::chrono::milliseconds(10);
        auto timeWhenToPublish = std::chrono::steady_clock::now();
        while (m_clockPluginDestructionOngoing) {
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
            ecm = &_ecm;

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

            if (!m_clockPort.open(m_portName))
            {
                yError() << "Failed to open port" << m_portName;
                return;
            }

            m_gzInstanceId = DeviceRegistry::getGzInstanceId(_ecm);
            DeviceRegistry::getHandler()->insertClockPlugin(_entity, _ecm, m_clockPluginID);
            m_initialized = true;
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
            auto resetYARPNetworkClockLambda
                = []() { yarp::os::NetworkBase::yarpClockInit(yarp::os::YARP_CLOCK_DEFAULT); };
            std::thread resetYARPNetworkClockThread(resetYARPNetworkClockLambda);
            resetYARPNetworkClockThread.detach();
            m_resetYARPClockAfterPortCreation = false;
        }

        publishTime(_info.simTime);
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

    // True between during clock plugin destruction and the first subsequent call to PostUpdate
    std::atomic<bool> m_clockPluginDestructionOngoing{false};

    // The gzInstanceId of the gz instance, assigned by the device registry
    std::string m_gzInstanceId;
    // The clock plugin ID assigned by the device registry, used when the clock plugin is removed
    std::string m_clockPluginID;

    // Backup of the ecm pointer, used in the destructor
    EntityComponentManager* ecm{nullptr};

};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::Clock,
              gz::sim::System,
              gzyarp::Clock::ISystemConfigure,
              gzyarp::Clock::ISystemPostUpdate)
