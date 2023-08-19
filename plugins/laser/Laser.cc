#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/Link.hh>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <iostream>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include<gz/transport/Node.hh>
#include <gz/sim/components/Lidar.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include "LaserDriver.cpp"


using namespace gz;
using namespace sim;
using namespace systems;


class GzYarpLaser
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
        
{
  public:
    
    GzYarpLaser() : m_deviceRegistered(false)
    {
    }
    
    virtual ~GzYarpLaser()
    {
      if (m_deviceRegistered) 
      {
          Handler::getHandler()->removeDevice(m_deviceScopedName);
          m_deviceRegistered = false;
      }
      
      if( m_laserDriver.isValid() ) m_laserDriver.close();
      HandlerLaser::getHandler()->removeSensor(sensorScopedName);
      yarp::os::Network::fini();
    }
    
    virtual void Configure(const Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          EntityComponentManager &_ecm,
                          EventManager &/*_eventMgr*/) override
    { 
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork())
        {
            yError() << "Yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GzYarpLaserDriver>
                                            ("gazebo_laser", "", "GzYarpLaserDriver"));
        ::yarp::os::Property driver_properties;

        bool wipe = false;
        if (_sdf->HasElement("yarpConfigurationFile")) 
        {
            std::string ini_file_path = _sdf->Get<std::string>("yarpConfigurationFile");            
            driver_properties.fromConfigFile(ini_file_path.c_str(),wipe);
            if (!driver_properties.check("sensorName"))
            {
                yError() << "gz-yarp-Laser : missing sensorName parameter";
                return;
            }
            if (!driver_properties.check("parentLinkName"))
            {
                yError() << "gz-yarp-Laser : missing parentLinkName parameter";
                return;
            }
            yInfo() << "gz-yarp-Laser: configuration of sensor " << driver_properties.find("sensorName").asString() 
                    << " loaded from yarpConfigurationFile : " << ini_file_path << "\n";
        }
        else 
        {
            yError() << "gz-yarp-Laser : missing yarpConfigurationFile element";
            return; 
        }
        std::string sensorName = driver_properties.find("sensorName").asString();
        std::string parentLinkName = driver_properties.find("parentLinkName").asString();
        auto model = Model(_entity);
        auto parentLink = model.LinkByName(_ecm, parentLinkName);
        this->sensor = _ecm.EntityByComponents(
            components::ParentEntity(parentLink),
            components::Name(sensorName),
            components::Sensor());
    
        sensorScopedName = scopedName(this->sensor, _ecm);
        this->laserData.sensorScopedName = sensorScopedName;

        driver_properties.put(YarpLaserScopedName.c_str(), sensorScopedName.c_str());
        if (!driver_properties.check("yarpDeviceName"))
        {
            yError() << "gz-yarp-Laser : missing yarpDeviceName parameter for device" << sensorScopedName;
            return;
        }

        //Insert the pointer in the singleton handler for retriving it in the yarp driver
        HandlerLaser::getHandler()->setSensor(&(this->laserData));

        driver_properties.put("device","gazebo_laser");
        driver_properties.put("sensor_name", sensorName);
        if( !m_laserDriver.open(driver_properties) ) 
        {
            yError()<<"gz-yarp-Laser Plugin failed: error in opening yarp driver";
            return;
        }

        m_deviceScopedName = sensorScopedName + "/" + driver_properties.find("yarpDeviceName").asString();

        if(!Handler::getHandler()->setDevice(m_deviceScopedName, &m_laserDriver))
        {
            yError()<<"gz-yarp-Laser: failed setting scopedDeviceName(=" << m_deviceScopedName << ")";
            return;
        }
        m_deviceRegistered = true;
        yInfo() << "Registered YARP device with instance name:" << m_deviceScopedName;
    }
    virtual void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override
    {
        if(!this->laserInitialized && _ecm.ComponentData<components::SensorTopic>(sensor).has_value())
        {
            this->laserInitialized = true;
            auto laserTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value();
            this->node.Subscribe(laserTopicName, &GzYarpLaser::laserCb, this);
        }
    }

    virtual void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override
    {
        gz::msgs::LaserScan laserMsg;
        {
            std::lock_guard<std::mutex> lock(this->laserMsgMutex);
            laserMsg = this->laserMsg;
        }

        std::lock_guard<std::mutex> lock(laserData.m_mutex);
        laserData.m_data.resize(laserMsg.ranges().size());

        for (size_t i=0; i< laserMsg.ranges().size(); i++)
        {
          laserData.m_data[i] = laserMsg.ranges(i);
        }
        
        laserData.simTime = _info.simTime.count()/1e9;
        
    }

    void laserCb(const gz::msgs::LaserScan &_msg)
    {
        std::lock_guard<std::mutex> lock(this->laserMsgMutex);
        laserMsg = _msg;
    }
    
  private: 
    Entity sensor;
    yarp::dev::PolyDriver m_laserDriver;
    std::string m_deviceScopedName;
    std::string sensorScopedName;
    bool m_deviceRegistered;
    LaserData laserData;
    bool laserInitialized;
    gz::transport::Node node;
    gz::msgs::LaserScan laserMsg;
    std::mutex laserMsgMutex;
};


 
// Register plugin
GZ_ADD_PLUGIN(GzYarpLaser,
                    gz::sim::System,
                    GzYarpLaser::ISystemConfigure,
                    GzYarpLaser::ISystemPreUpdate,
                    GzYarpLaser::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(GzYarpLaser, "gz::sim::systems::GzYarpLaser")
