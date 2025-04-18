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
    bool                  setRgbResolution(int width, int height) override;
    bool                  getRgbFOV(double& horizontalFov, double& verticalFov) override;
    bool                  setRgbFOV(double horizontalFov, double verticalFov) override;
    bool                  getRgbMirroring(bool& mirror) override;
    bool                  setRgbMirroring(bool mirror) override;
    bool                  getRgbIntrinsicParam(yarp::os::Property& intrinsic) override;
    bool                  getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp = NULL) override;

    int                   getDepthHeight() override;
    int                   getDepthWidth() override;
    bool                  setDepthResolution(int width, int height) override;
    bool                  getDepthFOV(double& horizontalFov, double& verticalFov) override;
    bool                  setDepthFOV(double horizontalFov, double verticalFov) override;
    bool                  getDepthIntrinsicParam(yarp::os::Property& intrinsic) override;
    double                getDepthAccuracy() override;
    bool                  setDepthAccuracy(double accuracy) override;
    bool                  getDepthClipPlanes(double& nearPlane, double& farPlane) override;
    bool                  setDepthClipPlanes(double nearPlane, double farPlane) override;
    bool                  getDepthMirroring(bool& mirror) override;
    bool                  setDepthMirroring(bool mirror) override;
    bool                  getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp = NULL) override;

    bool                  getExtrinsicParam(yarp::sig::Matrix &extrinsic) override;
    bool                  getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp=NULL, yarp::os::Stamp* depthStamp=NULL) override;
    RGBDSensor_status     getSensorStatus() override;
    std::string           getLastErrorMsg(yarp::os::Stamp* timeStamp = NULL) override;
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