#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Sensor.hh>
#include "gz/sim/components/ForceTorque.hh"
#include "gz/sim/components/JointTransmittedWrench.hh"
#include "gz/sim/components/Pose.hh"
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <iostream>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/dev/IMultipleWrapper.h>
#include "ForceTorqueDriver.cpp"
#include <sdf/ForceTorque.hh>


using namespace gz;
using namespace sim;
using namespace systems;


class ForceTorque
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  public:
    ForceTorque() : m_iWrap(0), m_deviceRegistered(false)
    {
    }
    
    virtual ~ForceTorque()
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
      auto model = Model(_entity);
      this->joint = model.JointByName(_ecm, "joint_12");
      this->sensor = Joint(this->joint).SensorByName(_ecm, "force_torque");
      std::string childLinkName = Joint(this->joint).ChildLinkName(_ecm).value();
      std::string parentLinkName = Joint(this->joint).ParentLinkName(_ecm).value();
      this->childLink = model.LinkByName(_ecm, childLinkName);
      this->parentLink = model.LinkByName(_ecm, parentLinkName);
      this->measureFrame = _ecm.Component<components::ForceTorque>(this->sensor)->Data().ForceTorqueSensor()->Frame();
      this->measureDirection = _ecm.Component<components::ForceTorque>(this->sensor)->Data().ForceTorqueSensor()->MeasureDirection();

      sensorScopedName = scopedName(this->sensor, _ecm);
      this->forceTorqueData.sensorScopedName = sensorScopedName;

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
      auto sdf = _ecm.Component<components::ForceTorque>(this->sensor)->Data().Element().get();
      std::string sensorName = Sensor(this->sensor).Name(_ecm).value();
      bool wipe = false;
      if (_sdf->HasElement("yarpConfigurationString"))
      {
          std::string configuration_string = _sdf->Get<std::string>("yarpConfigurationString");
          driver_properties.fromString(configuration_string, wipe);
          yInfo() << "GazeboYarpPlugins: configuration of sensor " << sensorName << " loaded from yarpConfigurationString : " << configuration_string << "\n";
      }
      
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
  

    virtual void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override
    {
      auto jointWrench = _ecm.Component<components::JointTransmittedWrench>(this->joint);
      if (nullptr == jointWrench)
      {
        return;
      }
      // Notation:
      // X_WJ: Pose of joint in world
      // X_WP: Pose of parent link in world
      // X_WC: Pose of child link in world
      // X_WS: Pose of sensor in world
      // X_SP: Pose of parent link in sensors frame
      // X_SC: Pose of child link in sensors frame
      const auto X_WP = worldPose(this->parentLink, _ecm);
      const auto X_WC = worldPose(this->childLink, _ecm);
      const auto X_CJ = _ecm.Component<components::Pose>(this->joint)->Data();
      auto X_WJ = X_WC * X_CJ;
      auto X_JS = _ecm.Component<components::Pose>(this->sensor)->Data();
      auto X_WS = X_WJ * X_JS;
      auto X_SP = X_WS.Inverse() * X_WP;
      auto X_SC = X_WS.Inverse() * X_WC;

      math::Vector3d force =
          X_JS.Rot().Inverse() * msgs::Convert(jointWrench->Data().force());

      math::Vector3d torque =
          X_JS.Rot().Inverse() *
              msgs::Convert(jointWrench->Data().torque()) -
          X_JS.Pos().Cross(force);
        gz::math::Vector3d measuredForce;
        gz::math::Vector3d measuredTorque;

      if (measureFrame == sdf::ForceTorqueFrame::PARENT)
      {
        measuredForce =
            X_SP.Rot().Inverse() * force;
        measuredTorque =
            X_SP.Rot().Inverse() * torque;
      }
      else if (measureFrame == sdf::ForceTorqueFrame::CHILD)
      {
        measuredForce =
            X_SC.Rot().Inverse() * force;
        measuredTorque =
            X_SC.Rot() * torque;
      }
      else if (measureFrame == sdf::ForceTorqueFrame::SENSOR)
      {
        measuredForce = force;
        measuredTorque = torque;
      }
      else
      {
        gzerr << "measureFrame must be PARENT_LINK, CHILD_LINK or SENSOR\n";
      }
    

      if (measureDirection ==
          sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT)
      {
        measuredForce *= -1;
        measuredTorque *= -1;
      }    
      std::lock_guard<std::mutex> lock(forceTorqueData.m_mutex);
      forceTorqueData.m_data[0] = (measuredForce.X() != 0) ? measuredForce.X() : 0;
      forceTorqueData.m_data[1] = (measuredForce.Y() != 0) ? measuredForce.Y() : 0;
      forceTorqueData.m_data[2] = (measuredForce.Z() != 0) ? measuredForce.Z() : 0;
      forceTorqueData.m_data[3] = (measuredTorque.X() != 0) ? measuredTorque.X() : 0;
      forceTorqueData.m_data[4] = (measuredTorque.Y() != 0) ? measuredTorque.Y() : 0;
      forceTorqueData.m_data[5] = (measuredTorque.Z() != 0) ? measuredTorque.Z() : 0;
      forceTorqueData.simTime = _info.simTime.count()/1e9;
    }
 
  private: 
    Entity sensor;
    Entity joint;
    Entity parentLink;
    Entity childLink;
    sdf::ForceTorqueMeasureDirection measureDirection;
    sdf::ForceTorqueFrame measureFrame;
    yarp::dev::PolyDriver m_forcetorqueWrapper;
    yarp::dev::PolyDriver m_forceTorqueDriver;
    yarp::dev::IMultipleWrapper* m_iWrap;
    std::string m_deviceScopedName;
    std::string sensorScopedName;
    bool m_deviceRegistered;
    ForceTorqueData forceTorqueData;

};


 
// Register plugin
GZ_ADD_PLUGIN(ForceTorque,
                    gz::sim::System,
                    ForceTorque::ISystemConfigure,
                    ForceTorque::ISystemPostUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(ForceTorque, "gz::sim::systems::ForceTorque")