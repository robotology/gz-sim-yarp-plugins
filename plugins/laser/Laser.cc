#include <ConfigurationHelpers.hh>
#include <DeviceRegistry.hh>
#include <LaserDriver.cpp>
#include <LaserShared.hh>

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs.hh>
#include <gz/msgs/details/laserscan.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Lidar.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

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

class Laser : public System,
              public ISystemConfigure,
              public ISystemPreUpdate,
              public ISystemPostUpdate

{
public:
    Laser()
        : m_deviceRegistered(false)
    {
    }

    virtual ~Laser()
    {
        if (m_deviceRegistered)
        {
            DeviceRegistry::getHandler()->removeDevice(*ecm, m_deviceId);
            m_deviceRegistered = false;
        }

        if (m_laserDriver.isValid())
            m_laserDriver.close();
        yarp::os::Network::fini();
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        yarp::os::Network::init();

        ::yarp::dev::Drivers::factory().add(
            new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::LaserDriver>("gazebo_laser",
                                                                               "",
                                                                               "LaserDriver"));
        ::yarp::os::Property driver_properties;

        gzyarp::PluginConfigureHelper configureHelper(_ecm);

        ecm = &_ecm;

        if (ConfigurationHelpers::loadPluginConfiguration(_sdf, driver_properties))
        {
            if (!driver_properties.check("sensorName"))
            {
                yError() << "gz-sim-yarp-laser-system : missing sensorName parameter";
                return;
            }
            if (!driver_properties.check("parentLinkName"))
            {
                yError() << "gz-sim-yarp-laser-system : missing parentLinkName parameter";
                return;
            }
            if (!driver_properties.check("yarpDeviceName"))
            {
                yError() << "gz-sim-yarp-laser-system : missing yarpDeviceName parameter";
                return;
            }
            yInfo() << "gz-sim-yarp-laser-system: configuration of sensor "
                    << driver_properties.find("sensorName").asString() << " loaded";
        } else
        {
            yError() << "gz-sim-yarp-laser-system : missing configuration";
            return;
        }
        std::string sensorName = driver_properties.find("sensorName").asString();
        std::string parentLinkName = driver_properties.find("parentLinkName").asString();
        auto model = Model(_entity);
        auto parentLink = model.LinkByName(_ecm, parentLinkName);
        this->sensor = _ecm.EntityByComponents(components::ParentEntity(parentLink),
                                               components::Name(sensorName),
                                               components::Sensor());

        sensorScopedName = scopedName(this->sensor, _ecm);
        this->laserData.sensorScopedName = sensorScopedName;

        driver_properties.put(YarpLaserScopedName.c_str(), sensorScopedName.c_str());

        driver_properties.put("device", "gazebo_laser");
        driver_properties.put("sensor_name", sensorName);
        if (!m_laserDriver.open(driver_properties))
        {
            yError() << "gz-sim-yarp-laser-system Plugin failed: error in opening yarp driver";
            return;
        }

        ILaserData* iLaserData = nullptr;
        auto viewOk = m_laserDriver.view(iLaserData);

        if (!viewOk || !iLaserData)
        {
            yError() << "gz-sim-yarp-laser-system Plugin failed: error in getting "
                        "ILaserData interface";
            return;
        }
        iLaserData->setLaserData(&laserData);

        auto yarpDeviceNamee = driver_properties.find("yarpDeviceName").asString();

        if (!DeviceRegistry::getHandler()
                 ->setDevice(_entity, _ecm, yarpDeviceNamee, &m_laserDriver, m_deviceId))
        {
            yError() << "gz-sim-yarp-laser-system: failed setting scopedDeviceName(=" << m_deviceId
                     << ")";
            return;
        }

        configureHelper.setConfigureIsSuccessful(true);
        m_deviceRegistered = true;
        yInfo() << "Registered YARP device with instance name:" << m_deviceId;
    }
    virtual void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override
    {
        if (!this->laserInitialized
            && _ecm.ComponentData<components::SensorTopic>(sensor).has_value())
        {
            this->laserInitialized = true;
            auto laserTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value();
            this->node.Subscribe(laserTopicName, &Laser::laserCb, this);
        }
    }

    virtual void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        gz::msgs::LaserScan laserMsg;
        {
            std::lock_guard<std::mutex> lock(this->laserMsgMutex);
            laserMsg = this->laserMsg;
        }

        std::lock_guard<std::mutex> lock(laserData.m_mutex);
        laserData.m_data.resize(laserMsg.ranges().size());

        for (size_t i = 0; i < laserMsg.ranges().size(); i++)
        {
            laserData.m_data[i] = laserMsg.ranges(i);
        }

        laserData.simTime = _info.simTime.count() / 1e9;
    }

    void laserCb(const gz::msgs::LaserScan& _msg)
    {
        std::lock_guard<std::mutex> lock(this->laserMsgMutex);
        laserMsg = _msg;
    }

private:
    Entity sensor;
    yarp::dev::PolyDriver m_laserDriver;
    std::string m_deviceId;
    std::string sensorScopedName;
    bool m_deviceRegistered;
    LaserData laserData;
    bool laserInitialized{false};
    gz::transport::Node node;
    gz::msgs::LaserScan laserMsg;
    std::mutex laserMsgMutex;
    EntityComponentManager* ecm;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::Laser,
              gz::sim::System,
              gzyarp::Laser::ISystemConfigure,
              gzyarp::Laser::ISystemPreUpdate,
              gzyarp::Laser::ISystemPostUpdate)
