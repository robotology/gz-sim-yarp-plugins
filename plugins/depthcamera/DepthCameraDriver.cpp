#include "DepthCameraDriver.h"


using namespace yarp::dev::gzyarp;


YARP_LOG_COMPONENT(GZDEPTH, "gz-sim-yarp-plugins.plugins.GzYarpDepthCamera")


bool DepthCameraDriver::open(yarp::os::Searchable& config)
{
    yCDebug(GZDEPTH) << "Opening Gazebo Yarp Depth Camera Driver";
    return true;
}

bool DepthCameraDriver::close()
{
    yCDebug(GZDEPTH) << "Closing Gazebo Yarp Depth Camera Driver";
    return true;
}

int DepthCameraDriver::getRgbHeight()
{
    return m_height;
}

int DepthCameraDriver::getRgbWidth()
{
    return m_width;
}

bool DepthCameraDriver::setRgbResolution(int width, int height)
{
    m_width = width;
    m_height = height;
    return true;
}

bool DepthCameraDriver::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    horizontalFov = 0.0;
    verticalFov = 0.0;
    return true;
}

bool DepthCameraDriver::setRgbFOV(double horizontalFov, double verticalFov)
{
    return true;
}

bool DepthCameraDriver::getRgbMirroring(bool& mirror)
{
    mirror = false;
    return true;
}

bool DepthCameraDriver::setRgbMirroring(bool mirror)
{
    return true;
}

bool DepthCameraDriver::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    return true;
}

bool DepthCameraDriver::getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp)
{
    return true;
}

int DepthCameraDriver::getDepthHeight()
{
    return m_height;
}

int DepthCameraDriver::getDepthWidth()
{
    return m_width;
}

bool DepthCameraDriver::setDepthResolution(int width, int height)
{
    m_width = width;
    m_height = height;
    return true;
}

bool DepthCameraDriver::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    horizontalFov = 0.0;
    verticalFov = 0.0;
    return true;
}

bool DepthCameraDriver::setDepthFOV(double horizontalFov, double verticalFov)
{
    return true;
}


bool DepthCameraDriver::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    return true;
}

double DepthCameraDriver::getDepthAccuracy()
{
    return 0.00001;
}

bool DepthCameraDriver::setDepthAccuracy(double accuracy)
{
    yCError(GZDEPTH)  << "impossible to set accuracy";
    return false;
}

bool DepthCameraDriver::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    nearPlane = 0.0;
    farPlane = 0.0;
    return true;
}

bool DepthCameraDriver::setDepthClipPlanes(double nearPlane, double farPlane)
{
    return true;
}

bool DepthCameraDriver::getDepthMirroring(bool& mirror)
{
    mirror = false;
    return true;
}

bool DepthCameraDriver::setDepthMirroring(bool mirror)
{
    return true;
}

bool DepthCameraDriver::getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp)
{
    return true;
}

bool DepthCameraDriver::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
{
    extrinsic.resize(4, 4);
    extrinsic.zero();
    extrinsic[1][1] = extrinsic[2][2] = extrinsic[3][3] = extrinsic[4][4] = 1;
    return true;
}

bool DepthCameraDriver::getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp, yarp::os::Stamp* depthStamp)
{
    return true;
}

yarp::dev::IRGBDSensor::RGBDSensor_status DepthCameraDriver::getSensorStatus()
{
    return yarp::dev::IRGBDSensor::RGBDSensor_status::RGBD_SENSOR_OK_IN_USE;
}

std::string DepthCameraDriver::getLastErrorMsg(yarp::os::Stamp* timeStamp)
{
    return "No error";
}
