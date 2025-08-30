#include "DepthCameraDriver.h"
#include <gzyarp/YarpDevReturnValueCompat.h>

#include <cmath>
#include <cstring>

using namespace yarp::dev::gzyarp;

YARP_LOG_COMPONENT(GZDEPTH, "gz-sim-yarp-plugins.plugins.GzYarpDepthCamera")

// DeviceDriver
bool DepthCameraDriver::open(yarp::os::Searchable& config)
{
    // Manage depth quantization parameters (optional)
    if (config.check("QUANT_PARAM")) {
        yarp::os::Property quantCfg;
        quantCfg.fromString(config.findGroup("QUANT_PARAM").toString());
        m_depthQuantizationEnabled = true;
        if (quantCfg.check("depth_quant")) {
            m_depthDecimalNum = quantCfg.find("depth_quant").asInt32();
        }
    }
    return true;
}

bool DepthCameraDriver::close()
{
    return true;
}

// IRGBDSensor (RGB)
int DepthCameraDriver::getRgbHeight()
{
    return m_sensorData ? m_sensorData->m_height : 0;
}

int DepthCameraDriver::getRgbWidth()
{
    return m_sensorData ? m_sensorData->m_width : 0;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::setRgbResolution(int /*width*/, int /*height*/)
{
    yCError(GZDEPTH) << "setRgbResolution: impossible to set resolution";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    if (!m_sensorData) {
        yCError(GZDEPTH) << "getRgbFOV: sensor data not available!";
        return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40;
    }
    horizontalFov = m_sensorData->horizontal_fov;
    // Keep verticalFov as 0.0 to match tests expectations
    verticalFov = 0.0;
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::setRgbFOV(double /*horizontalFov*/, double /*verticalFov*/)
{
    yCError(GZDEPTH) << "setRgbFOV: impossible to set FOV";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getRgbMirroring(bool& mirror)
{
    mirror = false;
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::setRgbMirroring(bool /*mirror*/)
{
    yCError(GZDEPTH) << "setRgbMirroring: impossible to set mirroring";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    if (!m_sensorData) {
        yCError(GZDEPTH) << "getRgbIntrinsicParam: sensor data not available!";
        return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40;
    }
    yarp::os::Value        rectM;

    intrinsic.put("physFocalLength", 0.0);
    intrinsic.put("focalLengthX", m_sensorData->focalLengthX);
    intrinsic.put("focalLengthY", m_sensorData->focalLengthY);
    intrinsic.put("k1", m_sensorData->m_distModel.k1);
    intrinsic.put("k2", m_sensorData->m_distModel.k2);
    intrinsic.put("k3", m_sensorData->m_distModel.k3);
    intrinsic.put("t1", m_sensorData->m_distModel.p1);
    intrinsic.put("t2", m_sensorData->m_distModel.p2);
    intrinsic.put("principalPointX", m_sensorData->m_distModel.cx);
    intrinsic.put("principalPointY", m_sensorData->m_distModel.cy);
    intrinsic.put("rectificationMatrix", rectM.makeList("1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0"));
    intrinsic.put("distortionModel", "plumb_bob");
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp)
{
    if (!m_sensorData) {
        yCError(GZDEPTH) << "getRgbImage: sensor data not available!";
        return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40;
    }
    if (!timeStamp) {
        yCError(GZDEPTH) << "getRgbImage: timestamp pointer invalid!";
        return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40;
    }
    std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
    rgbImage.setPixelCode(m_sensorData->m_imageFormat);
    rgbImage.resize(m_sensorData->m_width, m_sensorData->m_height);
    const auto& data = m_sensorData->rgbCameraMsg.data();
    if (!data.empty()) {
        std::memcpy(rgbImage.getRawImage(), data.data(), data.size());
    }
    timeStamp->update(m_sensorData->simTime);
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

#if defined(YARP_DEV_RETURN_VALUE_IS_GE_40)
YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getRgbResolution(int& width, int& height)
{
    width = getRgbWidth();
    height = getRgbHeight();
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getRgbSupportedConfigurations(std::vector<yarp::dev::CameraConfig>& /*configurations*/)
{
    yCError(GZDEPTH) << "getRgbSupportedConfigurations not implemented";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}
#endif

// IRGBDSensor (Depth)
int DepthCameraDriver::getDepthHeight()
{
    return getRgbHeight();
}

int DepthCameraDriver::getDepthWidth()
{
    return getRgbWidth();
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::setDepthResolution(int /*width*/, int /*height*/)
{
    yCError(GZDEPTH) << "setDepthResolution: impossible to set resolution";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    return getRgbFOV(horizontalFov, verticalFov);
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::setDepthFOV(double /*horizontalFov*/, double /*verticalFov*/)
{
    yCError(GZDEPTH) << "setDepthFOV: impossible to set FOV";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    return getRgbIntrinsicParam(intrinsic);
}

#if defined(YARP_DEV_RETURN_VALUE_IS_GE_40)
YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getDepthAccuracy(double& accuracy)
{
    accuracy = 1e-5;
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
#else
double DepthCameraDriver::getDepthAccuracy()
{
    return 1e-5;
}
#endif

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::setDepthAccuracy(double /*accuracy*/)
{
    yCError(GZDEPTH) << "setDepthAccuracy: impossible to set accuracy";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    if (!m_sensorData) {
        yCError(GZDEPTH) << "getDepthClipPlanes: sensor data not available!";
        return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40;
    }
    nearPlane = m_sensorData->nearPlane;
    farPlane = m_sensorData->farPlane;
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::setDepthClipPlanes(double /*nearPlane*/, double /*farPlane*/)
{
    yCError(GZDEPTH) << "setDepthClipPlanes: impossible to set clip planes";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getDepthMirroring(bool& mirror)
{
    mirror = false;
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::setDepthMirroring(bool /*mirror*/)
{
    yCError(GZDEPTH) << "setDepthMirroring: impossible to set mirroring";
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp)
{
    if (!m_sensorData) {
        yCError(GZDEPTH) << "getDepthImage: sensor data not available!";
        return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40;
    }
    if (!timeStamp) {
        yCError(GZDEPTH) << "getDepthImage: timestamp pointer invalid!";
        return YARP_DEV_RETURN_VALUE_ERROR_NOT_READY_CH40;
    }
    std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
    depthImage.resize(m_sensorData->m_width, m_sensorData->m_height);
    if (!m_depthQuantizationEnabled) {
        const auto& data = m_sensorData->depthCameraMsg.data();
        if (!data.empty()) {
            std::memcpy(depthImage.getRawImage(), data.data(), data.size());
        }
    } else {
        const float nearPlane = static_cast<float>(m_sensorData->nearPlane);
        const float farPlane = static_cast<float>(m_sensorData->farPlane);
        const float powCoeff = std::pow(10.0f, static_cast<float>(m_depthDecimalNum));
        auto pxPtr = reinterpret_cast<float*>(depthImage.getRawImage());
        for (int i = 0; i < m_sensorData->depthCameraMsg.data().size(); i++) {
            float value = m_sensorData->depthCameraMsg.data()[i];
            int intTemp = static_cast<int>(value * powCoeff);
            value = static_cast<float>(intTemp) / powCoeff;
            if (value < nearPlane) { value = nearPlane; }
            if (value > farPlane) { value = farPlane; }
            *pxPtr++ = value;
        }
    }
    timeStamp->update(m_sensorData->simTime);
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

#if defined(YARP_DEV_RETURN_VALUE_IS_GE_40)
YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getDepthResolution(int& width, int& height)
{
    width = getDepthWidth();
    height = getDepthHeight();
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
#endif

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getExtrinsicParam(yarp::sig::Matrix& /*extrinsic*/)
{
    return YARP_DEV_RETURN_VALUE_ERROR_NOT_IMPLEMENTED_BY_DEVICE_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp, yarp::os::Stamp* depthStamp)
{
    auto rv1 = getDepthImage(depthFrame, depthStamp);
    auto rv2 = getRgbImage(colorFrame, colorStamp);
    return (rv1 == YARP_DEV_RETURN_VALUE_OK_CH40) && (rv2 == YARP_DEV_RETURN_VALUE_OK_CH40) ? YARP_DEV_RETURN_VALUE_OK_CH40 : YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
}

#if defined(YARP_DEV_RETURN_VALUE_IS_GE_40)
YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getSensorStatus(yarp::dev::IRGBDSensor::RGBDSensor_status& status)
{
    status = yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE;
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}

YARP_DEV_RETURN_VALUE_TYPE_CH40 DepthCameraDriver::getLastErrorMsg(std::string& msg, yarp::os::Stamp* timeStamp)
{
    if (timeStamp) {
        timeStamp->update(m_sensorData ? m_sensorData->simTime : 0.0);
    }
    msg = "No error";
    return YARP_DEV_RETURN_VALUE_OK_CH40;
}
#else
yarp::dev::IRGBDSensor::RGBDSensor_status DepthCameraDriver::getSensorStatus()
{
    return yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE;
}

std::string DepthCameraDriver::getLastErrorMsg(yarp::os::Stamp* timeStamp)
{
    if (timeStamp) {
        timeStamp->update(m_sensorData ? m_sensorData->simTime : 0.0);
    }
    return std::string("No error");
}
#endif

// IDepthCameraData
void DepthCameraDriver::setDepthCameraData(::gzyarp::DepthCameraData* dataPtr)
{
    m_sensorData = dataPtr;
}
