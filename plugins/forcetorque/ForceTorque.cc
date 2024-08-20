#include <ConfigurationHelpers.hh>
#include <DeviceRegistry.hh>
#include <ForceTorqueDriver.cpp>

#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/details/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <sdf/ForceTorque.hh>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{

class ForceTorque : public System,
                    public ISystemConfigure,
                    public ISystemPreUpdate,
                    public ISystemPostUpdate
{
public:
    ForceTorque()
        : m_deviceRegistered(false)
    {
    }

    virtual ~ForceTorque()
    {
        if (m_deviceRegistered)
        {
            DeviceRegistry::getHandler()->removeDevice(*ecm, m_deviceId);
            m_deviceRegistered = false;
        }

        if (m_forceTorqueDriver.isValid())
            m_forceTorqueDriver.close();
        yarp::os::Network::fini();
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        yarp::os::Network::init();

        gzyarp::PluginConfigureHelper configureHelper(_ecm);

        ecm = &_ecm;

        std::string netWrapper = "analogServer";
        ::yarp::dev::Drivers::factory().add(
            new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::ForceTorqueDriver>("gazebo_"
                                                                                     "forcetorque",
                                                                                     netWrapper
                                                                                         .c_str(),
                                                                                     "ForceTorqueDr"
                                                                                     "iver"));

        ::yarp::os::Property driver_properties;

        if (ConfigurationHelpers::loadPluginConfiguration(_sdf, driver_properties))
        {
            if (!driver_properties.check("sensorName"))
            {
                yError() << "gz-sim-yarp-forcetorque-system : missing sensorName parameter";
                return;
            }
            if (!driver_properties.check("jointName"))
            {
                yError() << "gz-sim-yarp-forcetorque-system : missing jointName parameter";
                return;
            }
            if (!driver_properties.check("yarpDeviceName"))
            {
                yError() << "gz-sim-yarp-forcetorque-system : missing yarpDeviceName parameter";
                return;
            }
            yInfo() << "gz-sim-yarp-forcetorque-system: configuration of sensor "
                    << driver_properties.find("sensorName").asString() << " loaded";
        } else
        {
            yError() << "gz-sim-yarp-forcetorque-system : missing configuration";
            return;
        }

        std::string sensorName = driver_properties.find("sensorName").asString();
        std::string jointName = driver_properties.find("jointName").asString();
        auto model = Model(_entity);
        auto joint = model.JointByName(_ecm, jointName);
        this->sensor = Joint(joint).SensorByName(_ecm, sensorName);

        sensorScopedName = scopedName(this->sensor, _ecm);
        this->forceTorqueData.sensorScopedName = sensorScopedName;

        driver_properties.put(YarpForceTorqueScopedName.c_str(), sensorScopedName.c_str());

        driver_properties.put("device", "gazebo_forcetorque");
        driver_properties.put("sensor_name", sensorName);
        if (!m_forceTorqueDriver.open(driver_properties))
        {
            yError() << "gz-sim-yarp-forcetorque-system Plugin failed: error in opening yarp "
                        "driver";
            return;
        }

        IForceTorqueData* ftData = nullptr;
        auto viewOk = m_forceTorqueDriver.view(ftData);

        if (!viewOk || !ftData)
        {
            yError() << "gz-sim-yarp-forcetorque-system Plugin failed: error in getting "
                        "IForceTorqueData interface";
            return;
        }
        ftData->setForceTorqueData(&forceTorqueData);

        auto yarpDeviceName = driver_properties.find("yarpDeviceName").asString();

        if (!DeviceRegistry::getHandler()
                 ->setDevice(_entity, _ecm, yarpDeviceName, &m_forceTorqueDriver, m_deviceId))
        {
            yError() << "gz-sim-yarp-forcetorque-system: failed setting device with "
                        "scopedDeviceName(="
                     << m_deviceId << ") into DeviceRegistry";
            return;
        }

        configureHelper.setConfigureIsSuccessful(true);
        m_deviceRegistered = true;
        yInfo() << "Registered YARP device with instance name:" << m_deviceId;
    }

    virtual void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override
    {
        if (!this->ftInitialized && _ecm.ComponentData<components::SensorTopic>(sensor).has_value())
        {
            this->ftInitialized = true;
            auto ftTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value();
            this->node.Subscribe(ftTopicName, &ForceTorque::ftCb, this);
        }
    }

    virtual void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        gz::msgs::Wrench ftMsg;
        {
            std::lock_guard<std::mutex> lock(this->ftMsgMutex);
            ftMsg = this->ftMsg;
        }

        std::lock_guard<std::mutex> lock(forceTorqueData.m_mutex);
        forceTorqueData.m_data[0] = (ftMsg.force().x() != 0) ? ftMsg.force().x() : 0;
        forceTorqueData.m_data[1] = (ftMsg.force().y() != 0) ? ftMsg.force().y() : 0;
        forceTorqueData.m_data[2] = (ftMsg.force().z() != 0) ? ftMsg.force().z() : 0;
        forceTorqueData.m_data[3] = (ftMsg.torque().x() != 0) ? ftMsg.torque().x() : 0;
        forceTorqueData.m_data[4] = (ftMsg.torque().y() != 0) ? ftMsg.torque().y() : 0;
        forceTorqueData.m_data[5] = (ftMsg.torque().z() != 0) ? ftMsg.torque().z() : 0;
        forceTorqueData.simTime = _info.simTime.count() / 1e9;
    }

    void ftCb(const gz::msgs::Wrench& _msg)
    {
        std::lock_guard<std::mutex> lock(this->ftMsgMutex);
        ftMsg = _msg;
    }

private:
    Entity sensor;
    yarp::dev::PolyDriver m_forceTorqueDriver;
    std::string m_deviceId;
    std::string sensorScopedName;
    bool m_deviceRegistered;
    ForceTorqueData forceTorqueData;
    bool ftInitialized{false};
    gz::transport::Node node;
    gz::msgs::Wrench ftMsg;
    std::mutex ftMsgMutex;
    EntityComponentManager* ecm;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::ForceTorque,
              gz::sim::System,
              gzyarp::ForceTorque::ISystemConfigure,
              gzyarp::ForceTorque::ISystemPreUpdate,
              gzyarp::ForceTorque::ISystemPostUpdate)
