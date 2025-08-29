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
    yarp::dev::ReturnValue   getRgbResolution(int& width, int& height) override;
    yarp::dev::ReturnValue   setRgbResolution(int width, int height) override;
    yarp::dev::ReturnValue   getRgbFOV(double& horizontalFov, double& verticalFov) override;
    yarp::dev::ReturnValue   setRgbFOV(double horizontalFov, double verticalFov) override;
    yarp::dev::ReturnValue   getRgbMirroring(bool& mirror) override;
    yarp::dev::ReturnValue   setRgbMirroring(bool mirror) override;
    yarp::dev::ReturnValue   getRgbIntrinsicParam(yarp::os::Property& intrinsic) override;
    yarp::dev::ReturnValue   getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp = NULL) override;
    yarp::dev::ReturnValue   getRgbSupportedConfigurations(std::vector<yarp::dev::CameraConfig>& configurations) override;

    int                   getDepthHeight() override;
    int                   getDepthWidth() override;
    yarp::dev::ReturnValue   getDepthResolution(int& width, int& height) override;
    yarp::dev::ReturnValue   setDepthResolution(int width, int height) override;
    yarp::dev::ReturnValue   getDepthFOV(double& horizontalFov, double& verticalFov) override;
    yarp::dev::ReturnValue   setDepthFOV(double horizontalFov, double verticalFov) override;
    yarp::dev::ReturnValue   getDepthIntrinsicParam(yarp::os::Property& intrinsic) override;
    yarp::dev::ReturnValue   getDepthAccuracy(double& accuracy) override;
    yarp::dev::ReturnValue   setDepthAccuracy(double accuracy) override;
    yarp::dev::ReturnValue   getDepthClipPlanes(double& nearPlane, double& farPlane) override;
    yarp::dev::ReturnValue   setDepthClipPlanes(double nearPlane, double farPlane) override;
    yarp::dev::ReturnValue   getDepthMirroring(bool& mirror) override;
    yarp::dev::ReturnValue   setDepthMirroring(bool mirror) override;
    yarp::dev::ReturnValue   getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp = NULL) override;

    yarp::dev::ReturnValue   getExtrinsicParam(yarp::sig::Matrix &extrinsic) override;
    yarp::dev::ReturnValue   getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp=NULL, yarp::os::Stamp* depthStamp=NULL) override;
    yarp::dev::ReturnValue   getSensorStatus(RGBDSensor_status& status) override;
    yarp::dev::ReturnValue   getLastErrorMsg(std::string& msg, yarp::os::Stamp* timeStamp = NULL) override;

    
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
