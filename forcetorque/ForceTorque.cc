#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Sensor.hh>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <iostream>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/dev/IMultipleWrapper.h>
#include "ForceTorqueDriver.cpp"
#include <sdf/ForceTorque.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/components/Sensor.hh>


using namespace gz;
using namespace sim;
using namespace systems;


class GazeboYarpForceTorque
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
{
  public:
    GazeboYarpForceTorque() : m_iWrap(0), m_deviceRegistered(false)
    {
    }
    
    virtual ~GazeboYarpForceTorque()
    {
      if (m_deviceRegistered) 
      {
          Handler::getHandler()->removeDevice(m_deviceScopedName);
          m_deviceRegistered = false;
      }
      if(m_iWrap) { m_iWrap->detachAll(); m_iWrap = 0; }
      if( m_forcetorqueWrapper.isValid() ) m_forcetorqueWrapper.close();
      if( m_forceTorqueDriver.isValid() ) m_forceTorqueDriver.close();
      Handler::getHandler()->removeSensor(sensorScopedName);
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

      std::string netWrapper = "analogServer";
      ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpForceTorqueDriver>
                                        ("gazebo_forcetorque", netWrapper.c_str(), "GazeboYarpForceTorqueDriver"));
                                        
      ::yarp::os::Property driver_properties;

      bool wipe = false;
      if (_sdf->HasElement("yarpConfigurationString"))
      {
          std::string configuration_string = _sdf->Get<std::string>("yarpConfigurationString");
          driver_properties.fromString(configuration_string, wipe);
          yInfo() << "GazeboYarpPlugins: configuration of sensor " << driver_properties.find("sensor").asString() 
                  << " loaded from yarpConfigurationString : " << configuration_string << "\n";
      }
      
      std::string sensorName = driver_properties.find("sensor").asString();
      std::string jointName = driver_properties.find("joint").asString();
      auto model = Model(_entity);
      auto joint = model.JointByName(_ecm, jointName);
      this->sensor = Joint(joint).SensorByName(_ecm, sensorName);

      sensorScopedName = scopedName(this->sensor, _ecm);
      this->forceTorqueData.sensorScopedName = sensorScopedName;
      
      ::yarp::os::Property wrapper_properties = driver_properties;

      //Insert the pointer in the singleton handler for retriving it in the yarp driver
      Handler::getHandler()->setSensor(&(this->forceTorqueData));
      driver_properties.put(YarpForceTorqueScopedName.c_str(), sensorScopedName.c_str());

      bool disable_wrapper = driver_properties.check("disableImplicitNetworkWrapper");
      if (!disable_wrapper) 
      {
          //Open the wrapper
          wrapper_properties.put("device","analogServer");
          if( !m_forcetorqueWrapper.open(wrapper_properties) ) 
          {
              yError()<<"GazeboYarpForceTorque Plugin failed: error in opening yarp driver wrapper";
              return;
          }
      }
      if (disable_wrapper && !driver_properties.check("yarpDeviceName"))
      {
          yError() << "GazeboYarpForceTorque : missing yarpDeviceName parameter for device" << sensorScopedName;
          return;
      }
      driver_properties.put("device","gazebo_forcetorque");
      driver_properties.put("sensor_name", sensorName);
      if( !m_forceTorqueDriver.open(driver_properties) ) 
      {
          yError()<<"GazeboYarpForceTorque Plugin failed: error in opening yarp driver";
          return;
      }

      if (!disable_wrapper) 
      {
        //Attach the driver to the wrapper
        ::yarp::dev::PolyDriverList driver_list;

        if( !m_forcetorqueWrapper.view(m_iWrap) )
        {
            yError() << "GazeboYarpForceTorque : error in loading wrapper";
            return;
        }

        driver_list.push(&m_forceTorqueDriver,"dummy");

        if( !m_iWrap->attachAll(driver_list) ) 
        {
            yError() << "GazeboYarpForceTorque : error in connecting wrapper and device ";
        }

        if(!driver_properties.check("yarpDeviceName"))
        {
            m_deviceScopedName = sensorScopedName + "/" + driver_list[0]->key;
        }
        else
        {
            m_deviceScopedName = sensorScopedName + "/" + driver_properties.find("yarpDeviceName").asString();
        }
      } 
      else 
      {
          m_deviceScopedName = sensorScopedName + "/" + driver_properties.find("yarpDeviceName").asString();
      }
      
      if(!Handler::getHandler()->setDevice(m_deviceScopedName, &m_forceTorqueDriver))
      {
          yError()<<"GazeboYarpForceTorque: failed setting scopedDeviceName(=" << m_deviceScopedName << ")";
          return;
      }
      m_deviceRegistered = true;
      yInfo() << "Registered YARP device with instance name:" << m_deviceScopedName;
    }

    virtual void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override
    {
        if(!this->ftInitialized && _ecm.ComponentData<components::SensorTopic>(sensor).has_value())
        {
            this->ftInitialized = true;
            auto imuTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value();
            this->node.Subscribe(imuTopicName, &GazeboYarpForceTorque::ftCb, this);
        }
    }
  

    virtual void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override
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
        forceTorqueData.simTime = _info.simTime.count()/1e9;
    }

    void ftCb(const gz::msgs::Wrench &_msg)
    {
        std::lock_guard<std::mutex> lock(this->ftMsgMutex);
        ftMsg = _msg;
    }
 
  private: 
    Entity sensor;
    yarp::dev::PolyDriver m_forcetorqueWrapper;
    yarp::dev::PolyDriver m_forceTorqueDriver;
    yarp::dev::IMultipleWrapper* m_iWrap;
    std::string m_deviceScopedName;
    std::string sensorScopedName;
    bool m_deviceRegistered;
    ForceTorqueData forceTorqueData;
    bool ftInitialized;
    gz::transport::Node node;
    gz::msgs::Wrench ftMsg;
    std::mutex ftMsgMutex;

};


 
// Register plugin
GZ_ADD_PLUGIN(GazeboYarpForceTorque,
                    gz::sim::System,
                    GazeboYarpForceTorque::ISystemConfigure,
                    GazeboYarpForceTorque::ISystemPreUpdate,
                    GazeboYarpForceTorque::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(GazeboYarpForceTorque, "gz::sim::systems::GazeboYarpForceTorque")