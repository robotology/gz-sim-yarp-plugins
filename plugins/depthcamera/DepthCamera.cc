#include "DepthCamera.hh"
#include <gz/sensors/RgbdCameraSensor.hh>

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
}

void DepthCamera::Configure(const Entity& _entity,
                        const std::shared_ptr<const sdf::Element>& _sdf,
                        EntityComponentManager& _ecm,
                        EventManager& /*_eventMgr*/){

    gzyarp::PluginConfigureHelper configureHelper(_ecm);

    ecm = &_ecm;

    ::yarp::dev::Drivers::factory().add(
        new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::DepthCameraDriver>("gazebo_depth_camera",
                                                                            "rgbdSensor_nws_yarp",
                                                                            "DepthCameraDriver"));
    ::yarp::os::Property driver_properties;

    if (ConfigurationHelpers::loadPluginConfiguration(_sdf, driver_properties))
    {
        if (!driver_properties.check("sensorName"))
        {
            yError() << "gz-sim-yarp-depthcamera-system : missing sensorName parameter";
            return;
        }
        if (!driver_properties.check("parentLinkName"))
        {
            yError() << "gz-sim-yarp-depthcamera-system : missing parentLinkName parameter";
            return;
        }
        if (!driver_properties.check("yarpDeviceName"))
        {
            yError() << "gz-sim-yarp-depthcamera-system : missing yarpDeviceName parameter";
            return;
        }
        yInfo() << "gz-sim-yarp-depthcamera-system: configuration of sensor "
                << driver_properties.find("sensorName").asString() << " loaded";
    } else
    {
        yError() << "gz-sim-yarp-depthcamera-system : missing configuration";
        return;
    }

    std::string sensorName = driver_properties.find("sensorName").asString();
    std::string parentLinkName = driver_properties.find("parentLinkName").asString();

    auto model = Model(_entity);
    auto parentLink = model.LinkByName(_ecm, parentLinkName);
    this->sensor = _ecm.EntityByComponents(components::ParentEntity(parentLink),
                                           components::Name(sensorName),
                                           components::Sensor());
    
    auto sdfSensor       = _ecm.ComponentData<components::RgbdCamera>(sensor).value().Element();
    //auto A = sensorPippo->getDepthCamera();
    auto sdfCamera = sdfSensor->GetElement("camera").get();
    auto sdfImage  = sdfCamera->GetElement("image").get();

    auto sdfDistortion = sdfCamera->GetElement("distortion").get();
    auto sdfIntrinsics = sdfCamera->GetElement("lens").get()->GetElement("intrinsics").get();

    cameraData.init(sdfImage->Get<int>("width"), sdfImage->Get<int>("height"), sensorScopedName);
    // Let's get the camera data
    cameraData.horizontal_fov = sdfCamera->Get<double>("horizontal_fov");
    cameraData.vertical_fov   = sdfCamera->Get<double>("vertical_fov");
    cameraData.nearPlane      = sdfCamera->GetElement("clip").get()->Get<double>("near");
    cameraData.farPlane       = sdfCamera->GetElement("clip").get()->Get<double>("far");
    cameraData.focalLengthX   = sdfIntrinsics->Get<double>("fx");
    cameraData.focalLengthY   = sdfIntrinsics->Get<double>("fy");
    cameraData.m_distModel.k1 = sdfDistortion->Get<double>("k1");
    cameraData.m_distModel.k2 = sdfDistortion->Get<double>("k2");
    cameraData.m_distModel.k3 = sdfDistortion->Get<double>("k3");
    cameraData.m_distModel.p1 = sdfDistortion->Get<double>("p1");
    cameraData.m_distModel.p2 = sdfDistortion->Get<double>("p2");
    auto vectCenter = sdfDistortion->Get<gz::math::Vector2d>("center");
    cameraData.m_distModel.cx = vectCenter[0];
    cameraData.m_distModel.cy = vectCenter[1];
    driver_properties.put(YarpDepthCameraScopedName.c_str(), sensorScopedName.c_str());

    driver_properties.put("device", "gazebo_depth_camera");
    driver_properties.put("sensor_name", sensorName);

    // Open the driver
    if (!m_cameraDriver.open(driver_properties))
    {
        yError() << "gz-sim-yarp-depthcamera-system Plugin failed: error in opening yarp driver";
        return;
    }

    IDepthCameraData* iDepthCameraData = nullptr;
    auto viewOk = m_cameraDriver.view(iDepthCameraData);

    if (!viewOk || !iDepthCameraData)
    {
        yError() << "gz-sim-yarp-depthcamera-system Plugin failed: error in getting "
                    "IDepthCameraData interface";
        return;
    }
    iDepthCameraData->setDepthCameraData(&cameraData);

    m_cameraDriver.view(iRGBDSensor);
    if (iRGBDSensor == nullptr)
    {
        yError() << "Unable to get the IRGBDSensor interface from the device";
        return;
    }

    auto yarpDeviceName = driver_properties.find("yarpDeviceName").asString();

    if (!DeviceRegistry::getHandler()
                ->setDevice(_entity, _ecm, yarpDeviceName, &m_cameraDriver, m_deviceId))
    {
        yError() << "gz-sim-yarp-depthcamera-system: failed setting scopedDeviceName(=" << m_deviceId
                    << ")";
        return;
    }
    this->m_deviceRegistered = true;
    this->cameraInitialized = false;
    configureHelper.setConfigureIsSuccessful(true);
    yInfo() << "gz-sim-yarp-depthcamera-system: Registered YARP device with instance name:"
            << m_deviceId;
}

void DepthCamera::PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    if (!this->cameraInitialized
        && _ecm.ComponentData<components::SensorTopic>(sensor).has_value())
    {
        this->cameraInitialized = true;
        auto RgbCameraTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value() + "/image";
        auto DepthCameraTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value() + "/depth_image";
        auto ok = this->node.Subscribe(RgbCameraTopicName, &DepthCamera::RgbCameraCb, this);
        ok = ok & this->node.Subscribe(DepthCameraTopicName, &DepthCamera::DepthCameraCb, this);
        if (!ok)
        {
            yError() << "gz-sim-yarp-depthcamera-system: failed to subscribe rgb camera topic:"<<RgbCameraTopicName<<"depth camera topic:"<<DepthCameraTopicName;
        }
    }
}

void DepthCamera::PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm)
{
    if (this->cameraInitialized)
    {
        std::lock_guard<std::mutex> lock(cameraData.m_mutex);
        cameraData.simTime = _info.simTime.count() / 1e9;
    }
}

void DepthCamera::RgbCameraCb(const gz::msgs::Image& _msg)
{
    std::lock_guard<std::mutex> lock(this->cameraMsgMutex);
    cameraData.rgbCameraMsg = _msg;
    cameraData.m_imageFormat = m_format2VocabPixel.at(_msg.pixel_format_type());
}

void DepthCamera::DepthCameraCb(const gz::msgs::Image& _msg)
{
    std::lock_guard<std::mutex> lock(this->cameraMsgMutex);
    cameraData.depthCameraMsg = _msg;
    cameraData.m_depthFormat = m_format2VocabPixel.at(_msg.pixel_format_type());
}
