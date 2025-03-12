#include "DepthCameraDriver.h"


using namespace yarp::dev::gzyarp;


YARP_LOG_COMPONENT(GZDEPTH, "gz-sim-yarp-plugins.plugins.GzYarpDepthCamera")


bool DepthCameraDriver::open(yarp::os::Searchable& config)
{
    yCDebug(GZDEPTH) << "Opening Gazebo Yarp Depth Camera Driver";
    //Manage depth quantization parameter
    if(config.check("QUANT_PARAM")) {
        yarp::os::Property quantCfg;
        quantCfg.fromString(config.findGroup("QUANT_PARAM").toString());
        m_depthQuantizationEnabled = true;
        if (quantCfg.check("depth_quant")) {
            m_depthDecimalNum = quantCfg.find("depth_quant").asInt32();
        }
    }

    m_conf.fromString(config.toString());

    m_vertical_flip     = config.check("vertical_flip");
    m_horizontal_flip   = config.check("horizontal_flip");
    m_display_timestamp = config.check("display_timestamp");
    m_display_time_box  = config.check("display_time_box");

    return true;
}

bool DepthCameraDriver::close()
{
    yCDebug(GZDEPTH) << "Closing Gazebo Yarp Depth Camera Driver";
    return true;
}

int DepthCameraDriver::getRgbHeight()
{
    return m_sensorData->m_height;
}

int DepthCameraDriver::getRgbWidth()
{
    return m_sensorData->m_width;
}

bool DepthCameraDriver::setRgbResolution(int width, int height)
{
    yCError(GZDEPTH) << "setRgbResolution: impossible to set resolution";
    return false;
}

bool DepthCameraDriver::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "getRgbFOV: sensor data not available!";
        return false;
    }
    horizontalFov = m_sensorData->horizontal_fov;
    verticalFov   = m_sensorData->vertical_fov;
    return true;
}

bool DepthCameraDriver::setRgbFOV(double horizontalFov, double verticalFov)
{
    yCError(GZDEPTH) << "setRgbFOV: impossible to set FOV";
    return false;
}

bool DepthCameraDriver::getRgbMirroring(bool& mirror)
{
    mirror = false;
    return true;
}

bool DepthCameraDriver::setRgbMirroring(bool mirror)
{
    yCError(GZDEPTH) << "setRgbMirroring: impossible to set mirroring";
    return false;
}

bool DepthCameraDriver::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "getRgbIntrinsicParam: sensor data not available!";
        return false;
    }
    yarp::os::Value        rectM;

    intrinsic.put("physFocalLength", 0.0);

    intrinsic.put("focalLengthX",    m_sensorData->focalLengthX);
    intrinsic.put("focalLengthY",    m_sensorData->focalLengthY);

    intrinsic.put("k1",              m_sensorData->m_distModel.k1);
    intrinsic.put("k2",              m_sensorData->m_distModel.k2);
    intrinsic.put("k3",              m_sensorData->m_distModel.k3);
    intrinsic.put("t1",              m_sensorData->m_distModel.p1);
    intrinsic.put("t2",              m_sensorData->m_distModel.p2);
    intrinsic.put("principalPointX", m_sensorData->m_distModel.cx);
    intrinsic.put("principalPointY", m_sensorData->m_distModel.cy);
    intrinsic.put("rectificationMatrix", rectM.makeList("1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0"));
    intrinsic.put("distortionModel", "plumb_bob");
    intrinsic.put("stamp",           m_sensorData->simTime);
    return true;
}

bool DepthCameraDriver::getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "getRgbImage: sensor data not available!";
        return false;
    }
    if(!timeStamp)
    {
        yCError(GZDEPTH) << "getRgbImage: timestamp pointer invalid!";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
    rgbImage.setPixelCode(m_sensorData->m_imageFormat);
    rgbImage.resize(m_sensorData->m_width, m_sensorData->m_height);
    memcpy(rgbImage.getRawImage(), m_sensorData->rgbCameraMsg.data().c_str(), m_sensorData->rgbCameraMsg.ByteSizeLong());
    timeStamp->update(m_sensorData->simTime);
    // TODO vertical and horizontal flip, display timestamp and time box
    return true;
}

int DepthCameraDriver::getDepthHeight()
{
    return getRgbHeight();
}

int DepthCameraDriver::getDepthWidth()
{
    return getRgbWidth();
}

bool DepthCameraDriver::setDepthResolution(int width, int height)
{
    yCError(GZDEPTH) << "setDepthResolution: impossible to set resolution";
    return false;
}

bool DepthCameraDriver::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    return getRgbFOV(horizontalFov, verticalFov);
}

bool DepthCameraDriver::setDepthFOV(double horizontalFov, double verticalFov)
{
    yCError(GZDEPTH) << "setDepthFOV: impossible to set FOV";
    return false;
}


bool DepthCameraDriver::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    return getRgbIntrinsicParam(intrinsic);
}

double DepthCameraDriver::getDepthAccuracy()
{
    return 0.00001;
}

bool DepthCameraDriver::setDepthAccuracy(double accuracy)
{
    yCError(GZDEPTH)  << "setDepthAccuracy: impossible to set accuracy";
    return false;
}

bool DepthCameraDriver::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "getDepthClipPlanes: sensor data not available!";
        return false;
    }
    nearPlane = m_sensorData->nearPlane;
    farPlane  = m_sensorData->farPlane;
    return true;
}

bool DepthCameraDriver::setDepthClipPlanes(double nearPlane, double farPlane)
{
    yCError(GZDEPTH)  << "setDepthClipPlanes: impossible to set clip planes";
    return false;
}

bool DepthCameraDriver::getDepthMirroring(bool& mirror)
{
    mirror = false;
    return true;
}

bool DepthCameraDriver::setDepthMirroring(bool mirror)
{
    yCError(GZDEPTH)  << "setDepthMirroring: impossible to set mirroring";
    return false;
}

bool DepthCameraDriver::getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "gazeboDepthCameraDriver: sensor data not available!";
        return false;
    }
    if(!timeStamp)
    {
        yCError(GZDEPTH)  << "gazeboDepthCameraDriver: timestamp pointer invalid!";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
    depthImage.resize(m_sensorData->m_width, m_sensorData->m_height);
    if(!m_depthQuantizationEnabled) {
        memcpy(depthImage.getRawImage(), m_sensorData->depthCameraMsg.data().c_str(), m_sensorData->depthCameraMsg.ByteSizeLong());
    }
    else {
        float nearPlane = (float) m_sensorData->nearPlane;
        float farPlane = (float) m_sensorData->farPlane;
        int intTemp;
        float value;
        for (int i = 0; i < m_sensorData->depthCameraMsg.ByteSizeLong(); i++) {
            value = m_sensorData->depthCameraMsg.data()[i];
            intTemp = (int) ((value - nearPlane) / (farPlane - nearPlane) * 255);
            depthImage.getRawImage()[i] = intTemp;
        }
    }
    timeStamp->update(m_sensorData->simTime);
    //TODO vertical and horizontal flip, display timestamp and time box
    return true;
}

bool DepthCameraDriver::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
{
    extrinsic.resize(4, 4);
    extrinsic.zero();
    extrinsic[0][0] = extrinsic[1][1] = extrinsic[2][2] = extrinsic[3][3] = 1;
    return true;
}

bool DepthCameraDriver::getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp, yarp::os::Stamp* depthStamp)
{
    return getDepthImage(depthFrame, depthStamp) && getRgbImage(colorFrame, colorStamp);
}

yarp::dev::IRGBDSensor::RGBDSensor_status DepthCameraDriver::getSensorStatus()
{
    return yarp::dev::IRGBDSensor::RGBDSensor_status::RGBD_SENSOR_OK_IN_USE;
}

std::string DepthCameraDriver::getLastErrorMsg(yarp::os::Stamp* timeStamp)
{
    if(timeStamp)
    {
        timeStamp->update(m_sensorData->simTime);
    }
    return "No error";
}

void DepthCameraDriver::setDepthCameraData(::gzyarp::DepthCameraData* dataPtr)
{
    m_sensorData = dataPtr;
}
