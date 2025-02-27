#include "DepthCamera.hh"
using namespace gz;
using namespace sim;
using namespace systems;
using namespace gzyarp;


DepthCamera::DepthCamera()
    : m_deviceRegistered(false)
{
}

DepthCamera::~DepthCamera()
{
    if (m_deviceRegistered)
    {
        DeviceRegistry::getHandler()->removeDevice(*ecm, m_deviceId);
        m_deviceRegistered = false;
    }

    if (m_cameraDriver.isValid())
    {
        m_cameraDriver.close();
    }
    yarp::os::Network::fini();
}

void DepthCamera::Configure(const Entity& _entity,
                        const std::shared_ptr<const sdf::Element>& _sdf,
                        EntityComponentManager& _ecm,
                        EventManager& /*_eventMgr*/){
    yarp::os::Network::init();

    gzyarp::PluginConfigureHelper configureHelper(_ecm);

    ecm = &_ecm;

    ::yarp::dev::Drivers::factory().add(
        new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::DepthCameraDriver>("gazebo_camera",
                                                                            "grabber",
                                                                            "DepthCameraDriver"));
    ::yarp::os::Property driver_properties;

    if (ConfigurationHelpers::loadPluginConfiguration(_sdf, driver_properties))
    {
        if (!driver_properties.check("sensorName"))
        {
            yError() << "gz-sim-yarp-camera-system : missing sensorName parameter";
            return;
        }
        if (!driver_properties.check("parentLinkName"))
        {
            yError() << "gz-sim-yarp-camera-system : missing parentLinkName parameter";
            return;
        }
        if (!driver_properties.check("yarpDeviceName"))
        {
            yError() << "gz-sim-yarp-camera-system : missing yarpDeviceName parameter";
            return;
        }
        yInfo() << "gz-sim-yarp-camera-system: configuration of sensor "
                << driver_properties.find("sensorName").asString() << " loaded";
    } else
    {
        yError() << "gz-sim-yarp-camera-system : missing configuration";
        return;
    }

    std::string sensorName = driver_properties.find("sensorName").asString();
    std::string parentLinkName = driver_properties.find("parentLinkName").asString();

    auto model = Model(_entity);
    auto parentLink = model.LinkByName(_ecm, parentLinkName);
    this->sensor = _ecm.EntityByComponents(components::ParentEntity(parentLink),
                                            components::Name(sensorName),
                                            components::Sensor());
    auto sdfSensor = _ecm.ComponentData<components::RgbdCamera>(sensor).value().Element();
    auto sdfImage = sdfSensor.get()->GetElement("camera").get()->GetElement("image").get();

    cameraData.init(sdfImage->Get<int>("width"), sdfImage->Get<int>("height"), sensorScopedName);

    driver_properties.put(YarpDepthCameraScopedName.c_str(), sensorScopedName.c_str());

    driver_properties.put("device", "gazebo_camera");
    driver_properties.put("sensor_name", sensorName);

    // Open the driver
    if (!m_cameraDriver.open(driver_properties))
    {
        yError() << "gz-sim-yarp-camera-system Plugin failed: error in opening yarp driver";
        return;
    }

    IDepthCameraData* iDepthCameraData = nullptr;
    auto viewOk = m_cameraDriver.view(iDepthCameraData);

    if (!viewOk || !iDepthCameraData)
    {
        yError() << "gz-sim-yarp-camera-system Plugin failed: error in getting "
                    "IDepthCameraData interface";
        return;
    }
    iDepthCameraData->setDepthCameraData(&cameraData);

    m_cameraDriver.view(iFrameGrabberImage);
    if (iFrameGrabberImage == NULL)
    {
        yError() << "Unable to get the iFrameGrabberImage interface from the device";
        return;
    }

    auto yarpDeviceName = driver_properties.find("yarpDeviceName").asString();

    if (!DeviceRegistry::getHandler()
                ->setDevice(_entity, _ecm, yarpDeviceName, &m_cameraDriver, m_deviceId))
    {
        yError() << "gz-sim-yarp-camera-system: failed setting scopedDeviceName(=" << m_deviceId
                    << ")";
        return;
    }
    this->m_deviceRegistered = true;
    this->cameraInitialized = false;
    configureHelper.setConfigureIsSuccessful(true);
    yInfo() << "gz-sim-yarp-camera-system: Registered YARP device with instance name:"
            << m_deviceId;
}

void DepthCamera::PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    if (!this->cameraInitialized
        && _ecm.ComponentData<components::SensorTopic>(sensor).has_value())
    {
        this->cameraInitialized = true;
        auto DepthCameraTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value();
        this->node.Subscribe(DepthCameraTopicName, &DepthCamera::DepthCameraCb, this);
    }
}

void DepthCamera::PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm)
{
    gz::msgs::Image cameraMsg;
    {
        std::lock_guard<std::mutex> lock(this->cameraMsgMutex);
        cameraMsg = this->cameraMsg;
    }

    if (this->cameraInitialized)
    {
        std::lock_guard<std::mutex> lock(cameraData.m_mutex);
        //memcpy(cameraData.m_imageBuffer, cameraMsg.data().c_str(), cameraData.m_imageBufferSize);
        cameraData.simTime = _info.simTime.count() / 1e9;
    }
}

void DepthCamera::DepthCameraCb(const gz::msgs::Image& _msg)
{
    std::lock_guard<std::mutex> lock(this->cameraMsgMutex);
    cameraMsg = _msg;
}
