#include <DepthCameraDriver.h>
#include <DepthCameraShared.hh>
#include <DeviceRegistry.hh>
#include <gzyarp/ConfigurationHelpers.hh>

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/RgbdCamera.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>

#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{

namespace
{
struct PendingDepthCameraFrame
{
    std::mutex mutex;
    gz::msgs::Image rgbCameraMsg;
    gz::msgs::Image depthCameraMsg;
    bool hasRgbCameraMsg{false};
    bool hasDepthCameraMsg{false};
    std::uint64_t rgbSequence{0};
};

struct StereoDepthCameraEye
{
    Entity sensor{kNullEntity};
    yarp::dev::PolyDriver driver;
    std::string deviceId;
    std::string sensorName;
    std::string yarpDeviceName;
    DepthCameraData cameraData;
    PendingDepthCameraFrame pendingFrame;
    bool registered{false};
};

bool configureCameraData(const Entity& sensor,
                         EntityComponentManager& ecm,
                         DepthCameraData& cameraData)
{
    auto sdfSensS = ecm.ComponentData<components::RgbdCamera>(sensor);
    if (!sdfSensS.has_value()) {
        yError() << "gz-sim-yarp-stereodepthcamera-system : sensor is not of type='rgbd_camera'";
        return false;
    }

    auto sdfSensor = sdfSensS.value().Element();
    if (sdfSensor == nullptr) {
        yError() << "gz-sim-yarp-stereodepthcamera-system : sensor SDF element is nullptr";
        return false;
    }

    auto sdfCamera = sdfSensor->GetElement("camera").get();
    auto sdfImage = sdfCamera->GetElement("image").get();
    auto sdfDistortion = sdfCamera->GetElement("distortion").get();
    auto sdfIntrinsics = sdfCamera->GetElement("lens").get()->GetElement("intrinsics").get();

    cameraData.init(sdfImage->Get<int>("width"), sdfImage->Get<int>("height"), "");
    cameraData.horizontal_fov = sdfCamera->Get<double>("horizontal_fov");
    cameraData.vertical_fov = sdfCamera->Get<double>("vertical_fov");
    cameraData.nearPlane = sdfCamera->GetElement("clip").get()->Get<double>("near");
    cameraData.farPlane = sdfCamera->GetElement("clip").get()->Get<double>("far");
    cameraData.focalLengthX = sdfIntrinsics->Get<double>("fx");
    cameraData.focalLengthY = sdfIntrinsics->Get<double>("fy");
    cameraData.m_distModel.k1 = sdfDistortion->Get<double>("k1");
    cameraData.m_distModel.k2 = sdfDistortion->Get<double>("k2");
    cameraData.m_distModel.k3 = sdfDistortion->Get<double>("k3");
    cameraData.m_distModel.p1 = sdfDistortion->Get<double>("p1");
    cameraData.m_distModel.p2 = sdfDistortion->Get<double>("p2");
    auto vectCenter = sdfDistortion->Get<gz::math::Vector2d>("center");
    cameraData.m_distModel.cx = vectCenter[0];
    cameraData.m_distModel.cy = vectCenter[1];

    return true;
}

std::int64_t imageStampNs(const gz::msgs::Image& image)
{
    const auto& stamp = image.header().stamp();
    return (static_cast<std::int64_t>(stamp.sec()) * 1000000000LL) + stamp.nsec();
}
} // namespace

class StereoDepthCamera : public System,
                          public ISystemConfigure,
                          public ISystemPreUpdate,
                          public ISystemPostUpdate
{
public:
    StereoDepthCamera() = default;

    ~StereoDepthCamera() override
    {
        if (m_ecm != nullptr) {
            removeDevice(m_leftEye);
            removeDevice(m_rightEye);
        }

        if (m_leftEye.driver.isValid()) {
            m_leftEye.driver.close();
        }
        if (m_rightEye.driver.isValid()) {
            m_rightEye.driver.close();
        }

        if (m_stereoRgbPortOpen) {
            m_stereoRgbPort.close();
            m_stereoRgbPortOpen = false;
        }
    }

    void Configure(const Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   EntityComponentManager& ecm,
                   EventManager&) override
    {
        gzyarp::PluginConfigureHelper configureHelper(ecm);
        m_ecm = &ecm;

        yarp::dev::Drivers::factory().add(
            new yarp::dev::DriverCreatorOf<yarp::dev::gzyarp::DepthCameraDriver>(
                "gazebo_depth_camera", "rgbdSensor_nws_yarp", "DepthCameraDriver"));

        yarp::os::Property driverProperties;
        if (!ConfigurationHelpers::loadPluginConfiguration(sdf, driverProperties)) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : missing configuration";
            return;
        }

        const bool hasRequiredConfig =
            driverProperties.check("parentLinkName") &&
            driverProperties.check("leftSensorName") &&
            driverProperties.check("rightSensorName") &&
            driverProperties.check("leftYarpDeviceName") &&
            driverProperties.check("rightYarpDeviceName");

        if (!hasRequiredConfig) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : missing one of"
                     << "parentLinkName, leftSensorName, rightSensorName,"
                     << "leftYarpDeviceName, rightYarpDeviceName";
            return;
        }

        m_parentLinkName = driverProperties.find("parentLinkName").asString();
        m_leftEye.sensorName = driverProperties.find("leftSensorName").asString();
        m_rightEye.sensorName = driverProperties.find("rightSensorName").asString();
        m_leftEye.yarpDeviceName = driverProperties.find("leftYarpDeviceName").asString();
        m_rightEye.yarpDeviceName = driverProperties.find("rightYarpDeviceName").asString();

        if (driverProperties.check("stereoRgbPortName")) {
            m_stereoRgbPortName = driverProperties.find("stereoRgbPortName").asString();
        }

        if (!configureEye(entity, ecm, m_leftEye, "left") ||
            !configureEye(entity, ecm, m_rightEye, "right")) {
            return;
        }

        if (!m_stereoRgbPortName.empty()) {
            m_stereoRgbPortOpen = m_stereoRgbPort.open(m_stereoRgbPortName);
            if (!m_stereoRgbPortOpen) {
                yError() << "gz-sim-yarp-stereodepthcamera-system : failed opening"
                         << "side-by-side YARP RGB port" << m_stereoRgbPortName;
                return;
            }
        }

        configureHelper.setConfigureIsSuccessful(true);
        yInfo() << "gz-sim-yarp-stereodepthcamera-system: registered synchronized devices"
                << m_leftEye.deviceId << "and" << m_rightEye.deviceId;
        if (m_stereoRgbPortOpen) {
            yInfo() << "gz-sim-yarp-stereodepthcamera-system: side-by-side RGB port"
                    << m_stereoRgbPortName;
        }
    }

    void PreUpdate(const UpdateInfo&, EntityComponentManager& ecm) override
    {
        if (!m_leftSubscribed) {
            m_leftSubscribed = subscribeEye(ecm, m_leftEye, true);
        }
        if (!m_rightSubscribed) {
            m_rightSubscribed = subscribeEye(ecm, m_rightEye, false);
        }
    }

    void PostUpdate(const UpdateInfo& info, const EntityComponentManager&) override
    {
        const double simTime = info.simTime.count() / 1e9;
        publishEyeSnapshot(m_leftEye, simTime);
        publishEyeSnapshot(m_rightEye, simTime);
        publishStereoRgbImage(simTime);
    }

private:
    bool configureEye(const Entity& modelEntity,
                      EntityComponentManager& ecm,
                      StereoDepthCameraEye& eye,
                      const std::string& eyeName)
    {
        auto model = Model(modelEntity);
        auto parentLink = model.LinkByName(ecm, m_parentLinkName);
        eye.sensor = ecm.EntityByComponents(components::ParentEntity(parentLink),
                                            components::Name(eye.sensorName),
                                            components::Sensor());

        if (eye.sensor == kNullEntity) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : unable to find"
                     << eyeName << "sensor" << eye.sensorName;
            return false;
        }

        if (!configureCameraData(eye.sensor, ecm, eye.cameraData)) {
            return false;
        }

        yarp::os::Property properties;
        properties.put("device", "gazebo_depth_camera");
        properties.put("sensor_name", eye.sensorName);

        if (!eye.driver.open(properties)) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : failed opening"
                     << eyeName << "YARP driver";
            return false;
        }

        IDepthCameraData* depthCameraData = nullptr;
        const auto viewOk = eye.driver.view(depthCameraData);
        if (!viewOk || depthCameraData == nullptr) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : failed getting"
                     << eyeName << "IDepthCameraData interface";
            return false;
        }
        depthCameraData->setDepthCameraData(&eye.cameraData);

        yarp::dev::IRGBDSensor* rgbdSensor = nullptr;
        eye.driver.view(rgbdSensor);
        if (rgbdSensor == nullptr) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : failed getting"
                     << eyeName << "IRGBDSensor interface";
            return false;
        }

        if (!DeviceRegistry::getHandler()->setDevice(modelEntity,
                                                     ecm,
                                                     eye.yarpDeviceName,
                                                     &eye.driver,
                                                     eye.deviceId)) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : failed registering"
                     << eyeName << "device" << eye.yarpDeviceName;
            return false;
        }

        eye.registered = true;
        return true;
    }

    void removeDevice(StereoDepthCameraEye& eye)
    {
        if (eye.registered) {
            DeviceRegistry::getHandler()->removeDevice(*m_ecm, eye.deviceId);
            eye.registered = false;
        }
    }

    bool subscribeEye(EntityComponentManager& ecm,
                      StereoDepthCameraEye& eye,
                      const bool isLeft)
    {
        const auto topic = ecm.ComponentData<components::SensorTopic>(eye.sensor);
        if (!topic.has_value()) {
            return false;
        }

        const auto rgbCameraTopicName = topic.value() + "/image";
        const auto depthCameraTopicName = topic.value() + "/depth_image";

        bool ok = false;
        if (isLeft) {
            ok = m_node.Subscribe(rgbCameraTopicName,
                                  &StereoDepthCamera::LeftRgbCameraCb,
                                  this);
            ok = ok && m_node.Subscribe(depthCameraTopicName,
                                        &StereoDepthCamera::LeftDepthCameraCb,
                                        this);
        } else {
            ok = m_node.Subscribe(rgbCameraTopicName,
                                  &StereoDepthCamera::RightRgbCameraCb,
                                  this);
            ok = ok && m_node.Subscribe(depthCameraTopicName,
                                        &StereoDepthCamera::RightDepthCameraCb,
                                        this);
        }

        if (!ok) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : failed to subscribe"
                     << eye.sensorName << "topics" << rgbCameraTopicName
                     << depthCameraTopicName;
        }

        return ok;
    }

    void publishEyeSnapshot(StereoDepthCameraEye& eye, const double simTime)
    {
        gz::msgs::Image rgbCameraMsg;
        gz::msgs::Image depthCameraMsg;
        bool hasRgb = false;
        bool hasDepth = false;

        {
            std::lock_guard<std::mutex> lock(eye.pendingFrame.mutex);
            rgbCameraMsg = eye.pendingFrame.rgbCameraMsg;
            depthCameraMsg = eye.pendingFrame.depthCameraMsg;
            hasRgb = eye.pendingFrame.hasRgbCameraMsg;
            hasDepth = eye.pendingFrame.hasDepthCameraMsg;
        }

        std::lock_guard<std::mutex> lock(eye.cameraData.m_mutex);
        if (hasRgb) {
            eye.cameraData.rgbCameraMsg = rgbCameraMsg;
            eye.cameraData.m_imageFormat = m_format2VocabPixel.at(rgbCameraMsg.pixel_format_type());
        }
        if (hasDepth) {
            eye.cameraData.depthCameraMsg = depthCameraMsg;
            eye.cameraData.m_depthFormat = m_format2VocabPixel.at(depthCameraMsg.pixel_format_type());
        }
        eye.cameraData.simTime = simTime;
    }

    void setRgbCameraMsg(StereoDepthCameraEye& eye, const gz::msgs::Image& msg)
    {
        std::lock_guard<std::mutex> lock(eye.pendingFrame.mutex);
        eye.pendingFrame.rgbCameraMsg = msg;
        eye.pendingFrame.hasRgbCameraMsg = true;
        eye.pendingFrame.rgbSequence++;
    }

    void setDepthCameraMsg(StereoDepthCameraEye& eye, const gz::msgs::Image& msg)
    {
        std::lock_guard<std::mutex> lock(eye.pendingFrame.mutex);
        eye.pendingFrame.depthCameraMsg = msg;
        eye.pendingFrame.hasDepthCameraMsg = true;
    }

    void LeftRgbCameraCb(const gz::msgs::Image& msg)
    {
        setRgbCameraMsg(m_leftEye, msg);
    }

    void LeftDepthCameraCb(const gz::msgs::Image& msg)
    {
        setDepthCameraMsg(m_leftEye, msg);
    }

    void RightRgbCameraCb(const gz::msgs::Image& msg)
    {
        setRgbCameraMsg(m_rightEye, msg);
    }

    void RightDepthCameraCb(const gz::msgs::Image& msg)
    {
        setDepthCameraMsg(m_rightEye, msg);
    }

    void publishStereoRgbImage(const double simTime)
    {
        if (!m_stereoRgbPortOpen || m_stereoRgbPort.getOutputCount() == 0) {
            return;
        }

        gz::msgs::Image leftMsg;
        gz::msgs::Image rightMsg;
        std::uint64_t leftSequence = 0;
        std::uint64_t rightSequence = 0;

        {
            std::lock_guard<std::mutex> lock(m_leftEye.pendingFrame.mutex);
            if (!m_leftEye.pendingFrame.hasRgbCameraMsg) {
                return;
            }
            leftMsg = m_leftEye.pendingFrame.rgbCameraMsg;
            leftSequence = m_leftEye.pendingFrame.rgbSequence;
        }

        {
            std::lock_guard<std::mutex> lock(m_rightEye.pendingFrame.mutex);
            if (!m_rightEye.pendingFrame.hasRgbCameraMsg) {
                return;
            }
            rightMsg = m_rightEye.pendingFrame.rgbCameraMsg;
            rightSequence = m_rightEye.pendingFrame.rgbSequence;
        }

        if (leftSequence == m_lastStereoLeftSequence ||
            rightSequence == m_lastStereoRightSequence) {
            return;
        }

        const auto leftStampNs = imageStampNs(leftMsg);
        const auto rightStampNs = imageStampNs(rightMsg);
        if (leftStampNs != 0 && rightStampNs != 0 && leftStampNs != rightStampNs) {
            return;
        }

        if (leftMsg.width() != rightMsg.width() ||
            leftMsg.height() != rightMsg.height() ||
            leftMsg.pixel_format_type() != rightMsg.pixel_format_type()) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : unable to compose"
                     << "side-by-side image because left/right RGB formats differ";
            return;
        }

        const auto& leftData = leftMsg.data();
        const auto& rightData = rightMsg.data();
        const auto width = static_cast<std::size_t>(leftMsg.width());
        const auto height = static_cast<std::size_t>(leftMsg.height());
        if (width == 0 || height == 0 || leftData.empty() || rightData.empty()) {
            return;
        }
        if (leftData.size() != rightData.size() ||
            leftData.size() % (width * height) != 0) {
            yError() << "gz-sim-yarp-stereodepthcamera-system : unable to compose"
                     << "side-by-side image because RGB payload sizes are unexpected";
            return;
        }

        const auto bytesPerPixel = leftData.size() / (width * height);
        const auto sourceRowBytes = width * bytesPerPixel;

        auto& stereoImage = m_stereoRgbPort.prepare();
        stereoImage.setPixelCode(m_format2VocabPixel.at(leftMsg.pixel_format_type()));
        stereoImage.resize(static_cast<int>(2 * width), static_cast<int>(height));

        auto* destination = reinterpret_cast<unsigned char*>(stereoImage.getRawImage());
        const auto destinationRowBytes = static_cast<std::size_t>(stereoImage.getRowSize());

        for (std::size_t row = 0; row < height; ++row) {
            const auto sourceOffset = row * sourceRowBytes;
            auto* destinationRow = destination + (row * destinationRowBytes);
            std::memcpy(destinationRow, leftData.data() + sourceOffset, sourceRowBytes);
            std::memcpy(destinationRow + sourceRowBytes,
                        rightData.data() + sourceOffset,
                        sourceRowBytes);
        }

        yarp::os::Stamp stamp;
        stamp.update(simTime);
        m_stereoRgbPort.setEnvelope(stamp);
        m_stereoRgbPort.write();

        m_lastStereoLeftSequence = leftSequence;
        m_lastStereoRightSequence = rightSequence;
    }

    StereoDepthCameraEye m_leftEye;
    StereoDepthCameraEye m_rightEye;
    gz::transport::Node m_node;
    std::string m_parentLinkName;
    std::string m_stereoRgbPortName;
    yarp::os::BufferedPort<yarp::sig::FlexImage> m_stereoRgbPort;
    EntityComponentManager* m_ecm{nullptr};
    bool m_leftSubscribed{false};
    bool m_rightSubscribed{false};
    bool m_stereoRgbPortOpen{false};
    std::uint64_t m_lastStereoLeftSequence{0};
    std::uint64_t m_lastStereoRightSequence{0};
};

} // namespace gzyarp

GZ_ADD_PLUGIN(gzyarp::StereoDepthCamera,
              gz::sim::System,
              gzyarp::StereoDepthCamera::ISystemConfigure,
              gzyarp::StereoDepthCamera::ISystemPreUpdate,
              gzyarp::StereoDepthCamera::ISystemPostUpdate)
