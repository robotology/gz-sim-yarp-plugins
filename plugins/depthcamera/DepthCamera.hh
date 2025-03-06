#pragma once

#include <DepthCameraDriver.h>
#include <DepthCameraShared.hh>
#include <ConfigurationHelpers.hh>
#include <DeviceRegistry.hh>

#include <cstddef>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/details/image.pb.h>
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
#include <gz/sim/components/RgbdCamera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IRGBDSensor.h>
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
const std::string YarpDepthCameraScopedName = "sensorScopedName";

class DepthCamera : public System,
               public ISystemConfigure,
               public ISystemPreUpdate,
               public ISystemPostUpdate

{
public:
    DepthCamera();

    virtual ~DepthCamera();

    void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override;

    void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;

    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override;

    void RgbCameraCb(const gz::msgs::Image& _msg);
    void DepthCameraCb(const gz::msgs::Image& _msg);

private:
    Entity sensor;
    yarp::dev::PolyDriver m_cameraDriver;
    std::string m_deviceId;
    std::string sensorScopedName;
    
    bool m_deviceRegistered;
    DepthCameraData cameraData;
    bool cameraInitialized;
    gz::transport::Node node;
    gz::msgs::Image rgbCameraMsg;
    gz::msgs::Image depthCameraMsg;
    std::mutex cameraMsgMutex;
    yarp::dev::IRGBDSensor* iRGBDSensor;
    EntityComponentManager* ecm;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::DepthCamera,
              gz::sim::System,
              gzyarp::DepthCamera::ISystemConfigure,
              gzyarp::DepthCamera::ISystemPreUpdate,
              gzyarp::DepthCamera::ISystemPostUpdate)
