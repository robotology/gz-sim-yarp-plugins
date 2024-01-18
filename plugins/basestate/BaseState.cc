#include "BaseStateDriver.cpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <iostream>
// #include <sdf/BaseState.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{

class BaseState : public System,
                  public ISystemConfigure,
                  public ISystemPreUpdate,
                  public ISystemPostUpdate
{
public:
    BaseState()
        : m_deviceRegistered(false)
    {
    }

    virtual ~BaseState()
    {
        if (m_deviceRegistered)
        {
            Handler::getHandler()->removeDevice(m_deviceScopedName);
            m_deviceRegistered = false;
        }

        if (m_baseStateDriver.isValid())
            m_baseStateDriver.close();
        HandlerBaseState::getHandler()->removeModel(m_modelScopedName);
        yarp::os::Network::fini();
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork())
        {
            yError() << "Yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        std::string netWrapper = "analogServer";
        ::yarp::dev::Drivers::factory().add(
            new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::BaseStateDriver>("gazebo_"
                                                                                   "basestate",
                                                                                   netWrapper
                                                                                       .c_str(),
                                                                                   "BaseStateDrive"
                                                                                   "r"));

        ::yarp::os::Property driver_properties;

        bool wipe = false;
        if (_sdf->HasElement("yarpConfigurationString"))
        {
            std::string configuration_string = _sdf->Get<std::string>("yarpConfigurationString");
            driver_properties.fromString(configuration_string, wipe);
            if (!driver_properties.check("yarpDeviceName"))
            {
                yError() << "gz-sim-yarp-basestate-system : missing yarpDeviceName parameter";
                return;
            }
            if (!driver_properties.check("baseLink"))
            {
                yError() << "gz-sim-yarp-basestate-system : missing baseLink parameter";
                return;
            }
            yInfo() << "gz-sim-yarp-basestate-system: configuration of device "
                    << driver_properties.find("yarpDeviceName").asString()
                    << " loaded from yarpConfigurationString : " << configuration_string << "\n";
        } else
        {
            yError() << "gz-sim-yarp-basestate-system : missing yarpConfigurationString element";
            return;
        }

        std::string deviceName = driver_properties.find("yarpDeviceName").asString();
        std::string baseLinkName = driver_properties.find("baseLink").asString();

        auto model = Model(_entity);
        this->m_baseLink = model.LinkByName(_ecm, baseLinkName);

        m_modelScopedName = scopedName(model.Entity(), _ecm);
        this->m_baseStateData.m_modelScopedName = m_modelScopedName;

        driver_properties.put(YarpBaseStateScopedName.c_str(), m_modelScopedName.c_str());

        if (!driver_properties.check("yarpDeviceName"))
        {
            yError() << "gz-sim-yarp-basestate-system : missing yarpDeviceName parameter for "
                        "device "
                     << m_modelScopedName;
            return;
        }

        // Insert the pointer in the singleton handler for retrieving it in the yarp driver
        HandlerBaseState::getHandler()->setModel(&(this->m_baseStateData));

        driver_properties.put("device", "gazebo_basestate");
        driver_properties.put("robot", m_modelScopedName);

        if (!m_baseStateDriver.open(driver_properties))
        {
            yError() << "gz-sim-yarp-basestate-system Plugin failed: error in opening yarp "
                        "driver";
            return;
        }

        m_deviceScopedName
            = m_modelScopedName + "/" + driver_properties.find("yarpDeviceName").asString();

        if (!Handler::getHandler()->setDevice(m_deviceScopedName, &m_baseStateDriver))
        {
            yError() << "gz-sim-yarp-basestate-system: failed setting scopedDeviceName(="
                     << m_deviceScopedName << ")";
            return;
        }
        m_deviceRegistered = true;
        yInfo() << "Registered YARP device with instance name:" << m_deviceScopedName;
    }

    virtual void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override
    {
        if (!this->m_bsInitialized
            && _ecm.ComponentData<components::SensorTopic>(m_baseLink).has_value())
        {
            this->m_bsInitialized = true;
            auto ftTopicName = _ecm.ComponentData<components::SensorTopic>(m_baseLink).value();
            this->m_node.Subscribe(ftTopicName, &BaseState::ftCb, this);
        }
    }

    virtual void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        /*
        gz::msgs::Wrench m_bsMsg;
        {
            std::lock_guard<std::mutex> lock(this->m_msgMutex);
            m_bsMsg = this->m_bsMsg;
        }

        std::lock_guard<std::mutex> lock(forceTorqueData.m_mutex);
        forceTorqueData.m_data[0] = (m_bsMsg.force().x() != 0) ? m_bsMsg.force().x() : 0;
        forceTorqueData.m_data[1] = (m_bsMsg.force().y() != 0) ? m_bsMsg.force().y() : 0;
        forceTorqueData.m_data[2] = (m_bsMsg.force().z() != 0) ? m_bsMsg.force().z() : 0;
        forceTorqueData.m_data[3] = (m_bsMsg.torque().x() != 0) ? m_bsMsg.torque().x() : 0;
        forceTorqueData.m_data[4] = (m_bsMsg.torque().y() != 0) ? m_bsMsg.torque().y() : 0;
        forceTorqueData.m_data[5] = (m_bsMsg.torque().z() != 0) ? m_bsMsg.torque().z() : 0;
        forceTorqueData.m_simTime = _info.m_simTime.count() / 1e9;*/
    }

    void ftCb(const gz::msgs::Wrench& _msg)
    {
        std::lock_guard<std::mutex> lock(this->m_msgMutex);
        m_bsMsg = _msg;
    }

private:
    Entity m_baseLink;
    yarp::dev::PolyDriver m_baseStateDriver;
    std::string m_deviceScopedName;
    std::string m_modelScopedName;
    bool m_deviceRegistered;
    BaseStateData m_baseStateData;
    bool m_bsInitialized;
    gz::transport::Node m_node;
    gz::msgs::Wrench m_bsMsg;
    std::mutex m_msgMutex;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::BaseState,
              gz::sim::System,
              gzyarp::BaseState::ISystemConfigure,
              gzyarp::BaseState::ISystemPreUpdate,
              gzyarp::BaseState::ISystemPostUpdate)
