#include "DepthCameraDriver.h"


using namespace yarp::dev;
using namespace yarp::dev::gzyarp;


YARP_LOG_COMPONENT(GZDEPTH, "gz-sim-yarp-plugins.plugins.GzYarpDepthCamera")


bool DepthCameraDriver::open(yarp::os::Searchable& config)
{
    //Manage depth quantization parameter
    if(config.check("QUANT_PARAM")) {
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

int DepthCameraDriver::getRgbHeight()
{
    return m_sensorData->m_height;
}

int DepthCameraDriver::getRgbWidth()
{
    return m_sensorData->m_width;
}

ReturnValue DepthCameraDriver::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "getRgbFOV: sensor data not available!";
        return ReturnValue::return_code::return_value_error_not_ready;
    }
    horizontalFov = m_sensorData->horizontal_fov;
    verticalFov   = m_sensorData->vertical_fov;
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::setRgbFOV(double horizontalFov, double verticalFov)
{
    yCError(GZDEPTH) << "setRgbFOV: impossible to set FOV";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue DepthCameraDriver::getRgbMirroring(bool& mirror)
{
    mirror = false;
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::setRgbMirroring(bool mirror)
{
    yCError(GZDEPTH) << "setRgbMirroring: impossible to set mirroring";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue DepthCameraDriver::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "getRgbIntrinsicParam: sensor data not available!";
        return ReturnValue::return_code::return_value_error_not_ready;
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
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "getRgbImage: sensor data not available!";
        return ReturnValue::return_code::return_value_error_not_ready;
    }
    if(!timeStamp)
    {
        yCError(GZDEPTH) << "getRgbImage: timestamp pointer invalid!";
        return ReturnValue::return_code::return_value_error_not_ready;
    }
    std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
    rgbImage.setPixelCode(m_sensorData->m_imageFormat);
    rgbImage.resize(m_sensorData->m_width, m_sensorData->m_height);
    memcpy(rgbImage.getRawImage(), m_sensorData->rgbCameraMsg.data().c_str(), m_sensorData->rgbCameraMsg.ByteSizeLong());
    timeStamp->update(m_sensorData->simTime);
    return ReturnValue_ok;
}

int DepthCameraDriver::getDepthHeight()
{
    return getRgbHeight();
}

int DepthCameraDriver::getDepthWidth()
{
    return getRgbWidth();
}

ReturnValue DepthCameraDriver::setDepthResolution(int width, int height)
{
    yCError(GZDEPTH) << "setDepthResolution: impossible to set resolution";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue DepthCameraDriver::getDepthResolution(int& width, int& height)
{
    width=getRgbWidth();
    height=getDepthHeight();
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::setRgbResolution(int width, int height)
{
    yCError(GZDEPTH) << "setRgbResolution: impossible to set resolution";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue DepthCameraDriver::getRgbResolution(int& width, int& height)
{
    width=getRgbWidth();
    height=getDepthHeight();
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    return getRgbFOV(horizontalFov, verticalFov);
}

ReturnValue DepthCameraDriver::setDepthFOV(double horizontalFov, double verticalFov)
{
    yCError(GZDEPTH) << "setDepthFOV: impossible to set FOV";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}


ReturnValue DepthCameraDriver::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    return getRgbIntrinsicParam(intrinsic);
}

ReturnValue DepthCameraDriver::getDepthAccuracy(double& accuracy)
{
    accuracy = 0.00001;
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::setDepthAccuracy(double accuracy)
{
    yCError(GZDEPTH)  << "setDepthAccuracy: impossible to set accuracy";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue DepthCameraDriver::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "getDepthClipPlanes: sensor data not available!";
        return ReturnValue::return_code::return_value_error_not_ready;
    }
    nearPlane = m_sensorData->nearPlane;
    farPlane  = m_sensorData->farPlane;
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::setDepthClipPlanes(double nearPlane, double farPlane)
{
    yCError(GZDEPTH)  << "setDepthClipPlanes: impossible to set clip planes";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue DepthCameraDriver::getDepthMirroring(bool& mirror)
{
    mirror = false;
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::setDepthMirroring(bool mirror)
{
    yCError(GZDEPTH)  << "setDepthMirroring: impossible to set mirroring";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue DepthCameraDriver::getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp)
{
    if(!m_sensorData)
    {
        yCError(GZDEPTH) << "gazeboDepthCameraDriver: sensor data not available!";
        return ReturnValue::return_code::return_value_error_not_ready;
    }
    if(!timeStamp)
    {
        yCError(GZDEPTH)  << "gazeboDepthCameraDriver: timestamp pointer invalid!";
        return ReturnValue::return_code::return_value_error_not_ready;
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
        float powCoeff = pow(10.0f, (float) m_depthDecimalNum);

        auto pxPtr = reinterpret_cast<float*>(depthImage.getRawImage());
        for(int i=0; i< m_sensorData->depthCameraMsg.data().size(); i++){
            value = m_sensorData->depthCameraMsg.data()[i];

            intTemp = (int) (value * powCoeff);
            value = ((float) intTemp) / powCoeff;

            if (value < nearPlane) { value = nearPlane; }
            if (value > farPlane) { value = farPlane; }

            *pxPtr = value;
            pxPtr++;
        }
    }
    timeStamp->update(m_sensorData->simTime);
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
{
    yCError(GZDEPTH)  << "setDepthMirroring: impossible to getExtrinsicParam";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue DepthCameraDriver::getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp, yarp::os::Stamp* depthStamp)
{
    return getDepthImage(depthFrame, depthStamp) && getRgbImage(colorFrame, colorStamp);
}

ReturnValue DepthCameraDriver::getSensorStatus(yarp::dev::IRGBDSensor::RGBDSensor_status& status)
{
    status =  yarp::dev::IRGBDSensor::RGBDSensor_status::RGBD_SENSOR_OK_IN_USE;
    return ReturnValue_ok;
}

ReturnValue DepthCameraDriver::getLastErrorMsg(std::string& msg, yarp::os::Stamp* timeStamp)
{
    if(timeStamp)
    {
        timeStamp->update(m_sensorData->simTime);
    }
    msg="No error";
    return ReturnValue_ok;
}

void DepthCameraDriver::setDepthCameraData(::gzyarp::DepthCameraData* dataPtr)
{
    m_sensorData = dataPtr;
}

ReturnValue DepthCameraDriver::getRgbSupportedConfigurations(std::vector<yarp::dev::CameraConfig>& configurations)
{
    yCError(GZDEPTH)  << "setDepthMirroring: impossible to getRgbSupportedConfigurations";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}
