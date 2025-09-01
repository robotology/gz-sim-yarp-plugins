#pragma once

#include <DepthCameraShared.hh>
#include <DeviceRegistry.hh>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <gzyarp/YarpDevReturnValueCompat.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>


namespace yarp::dev::gzyarp
{
class DepthCameraDriver : public yarp::dev::DeviceDriver,
                          public yarp::dev::IRGBDSensor,
                          public ::gzyarp::IDepthCameraData {
public:
    DepthCameraDriver() = default;
    virtual ~DepthCameraDriver() = default;
    // yarp::dev::DeviceDriver
    bool                  open(yarp::os::Searchable& config) override;
    bool                  close() override;
    //IRGBDSensor
    int                   getRgbHeight() override;
    int                   getRgbWidth() override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRgbResolution(int width, int height) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRgbFOV(double& horizontalFov, double& verticalFov) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRgbFOV(double horizontalFov, double verticalFov) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRgbMirroring(bool& mirror) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setRgbMirroring(bool mirror) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRgbIntrinsicParam(yarp::os::Property& intrinsic) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp = NULL) override;
#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    // New in YARP 4
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRgbResolution(int& width, int& height) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getRgbSupportedConfigurations(std::vector<yarp::dev::CameraConfig>& configurations) override;
#endif

    int                   getDepthHeight() override;
    int                   getDepthWidth() override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setDepthResolution(int width, int height) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getDepthFOV(double& horizontalFov, double& verticalFov) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setDepthFOV(double horizontalFov, double verticalFov) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getDepthIntrinsicParam(yarp::os::Property& intrinsic) override;
#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getDepthAccuracy(double& accuracy) override;
#else
    double                getDepthAccuracy() override;
#endif
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setDepthAccuracy(double accuracy) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getDepthClipPlanes(double& nearPlane, double& farPlane) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setDepthClipPlanes(double nearPlane, double farPlane) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getDepthMirroring(bool& mirror) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 setDepthMirroring(bool mirror) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp = NULL) override;
#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getDepthResolution(int& width, int& height) override;
#endif

    YARP_DEV_RETURN_VALUE_TYPE_CH40 getExtrinsicParam(yarp::sig::Matrix &extrinsic) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp=NULL, yarp::os::Stamp* depthStamp=NULL) override;
#if (YARP_VERSION_MAJOR > 3) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR > 12) || (YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 12 && YARP_VERSION_PATCH >= 100)
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getSensorStatus(yarp::dev::IRGBDSensor::RGBDSensor_status& status) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH40 getLastErrorMsg(std::string& msg, yarp::os::Stamp* timeStamp = NULL) override;
#else
    RGBDSensor_status     getSensorStatus() override;
    std::string           getLastErrorMsg(yarp::os::Stamp* timeStamp = NULL) override;
#endif
    // IDepthCameraData
    void                  setDepthCameraData(::gzyarp::DepthCameraData* dataPtr) override;
private:
    int                 m_width{0};
    int                 m_height{0};

    int                 m_counter{0};
    std::string         m_error{""};

    ::gzyarp::DepthCameraData* m_sensorData;

    // Data quantization related parameters
    bool                m_depthQuantizationEnabled{false};
    int                 m_depthDecimalNum{0};


};

} // namespace yarp::dev::gzyarp