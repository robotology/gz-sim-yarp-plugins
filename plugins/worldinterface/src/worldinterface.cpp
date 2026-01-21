#include "worldinterface.h"

#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace gzyarp;


WorldInterface::WorldInterface()
{
}

WorldInterface::~WorldInterface()
{
    if (m_deviceRegistered)
    {
        DeviceRegistry::getHandler()->removeDevice(*m_ecm, m_deviceId);
        m_deviceRegistered = false;
    }

    if (m_worldDriver.isValid())
    {
        m_worldDriver.close();
    }
}

void WorldInterface::Configure(const Entity& _entity,
                        const std::shared_ptr<const sdf::Element>& _sdf,
                        EntityComponentManager& _ecm,
                        EventManager& /*_eventMgr*/)
{
    gzyarp::PluginConfigureHelper configureHelper(_ecm);
    m_ecm = &_ecm;
    
    ::yarp::dev::Drivers::factory().add(
        new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::WorldProxy>("worldProxy_device",
                                                                          "simulatedWorld_nws_yarp",
                                                                          "WorldProxy"));

    // Open the driver
    ::yarp::os::Property    driver_properties;
    driver_properties.put("device", "worldProxy_device");
    if (!m_worldDriver.open(driver_properties))
    {
        yError() << "gz-sim-yarp-worldinterface-system Plugin failed: error in opening yarp driver";
        return;
    }

    yarp::dev::gzyarp::IWorldSharedInterface* iws = nullptr;
    auto viewOk = m_worldDriver.view(iws);

    if (!viewOk || !iws)
    {
        yError() << "gz-sim-yarp-worldinterface-system Plugin failed: error in getting "
                    "IWorldSharedInterface interface";
        return;
    }
    iws->setNode(&m_node);

    std::string yarpDeviceName = "worldInterfaceName";
    if (driver_properties.check("yarpDeviceName"))
    {
        yarpDeviceName = driver_properties.find("yarpDeviceName").asString();
    }
    if (!DeviceRegistry::getHandler()->setDevice(_entity, _ecm, yarpDeviceName, &m_worldDriver, m_deviceId))
    {
        yError() << "gz-sim-yarp-worldinterface-system: failed setting scopedDeviceName(=" << m_deviceId
                    << ")";
        return;
    }
    
    configureHelper.setConfigureIsSuccessful(true);
    this->m_deviceRegistered = true;
    yInfo() << "Registered YARP device with instance name:" << m_deviceId;
    
}

void WorldInterface::PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    static int count =0;
    static yarp::sig::Pose6D pose;

        return;
  //  if (m_done) return;
  //  m_done = true;
yarp::dev::ISimulatedWorld* iss = nullptr;
    bool viewOk = m_worldDriver.view(iss);
    
///////
    if (count ==0)
    {
    
        double w = 1;
        double h = 1;
        double t = 1;

        yarp::sig::ColorRGB color;
        std::string frame_name;
        bool gravity_enable = 1;
        bool collision_enable = 1;
        
        iss->makeBox ("zzz", w, h, t, pose, color, frame_name, gravity_enable, collision_enable);
    }
    else
    {
        pose.x = pose.x += 0.001;
        pose.z = pose.z += 0.001;
        iss->setPose ("zzz", pose, "");
    }
    count ++;    
}

void WorldInterface::PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm)
{
   
}

