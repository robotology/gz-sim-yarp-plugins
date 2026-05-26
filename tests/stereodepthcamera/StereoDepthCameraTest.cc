#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/TestFixture.hh>

#include <gzyarp/YarpDevReturnValueCompat.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>

namespace
{
bool openClient(yarp::dev::PolyDriver& driver,
                const std::string& localPrefix,
                const std::string& remotePrefix)
{
    yarp::os::Property options;
#if defined(YARP_DEV_RETURN_VALUE_IS_GE_40)
    options.put("device", "RGBDSensor_nwc_yarp");
#else
    options.put("device", "RGBDSensorClient");
#endif
    options.put("localImagePort", localPrefix + "/rgb:i");
    options.put("localDepthPort", localPrefix + "/depth:i");
    options.put("remoteImagePort", remotePrefix + "/rgbImage:o");
    options.put("remoteDepthPort", remotePrefix + "/depthImage:o");
    options.put("localRpcPort", localPrefix + "/rpc:o");
    options.put("remoteRpcPort", remotePrefix + "/rpc:i");
    return driver.open(options);
}

bool readImages(yarp::dev::IRGBDSensor& sensor,
                yarp::sig::FlexImage& rgb,
                yarp::sig::ImageOf<yarp::sig::PixelFloat>& depth)
{
    for (std::size_t attempt = 0; attempt < 20; ++attempt) {
        yarp::os::Stamp rgbStamp;
        yarp::os::Stamp depthStamp;
        if (sensor.getImages(rgb, depth, &rgbStamp, &depthStamp)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    return false;
}
} // namespace

TEST(StereoDepthCameraTest, PublishesBothEyesAndSideBySideRgb)
{
    yarp::os::NetworkBase::setLocalMode(true);
    gz::common::Console::SetVerbosity(4);

    const auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model.sdf";
    gz::sim::TestFixture fixture(modelPath.string());
    fixture.Finalize();

    constexpr int iterations = 1000;
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::dev::PolyDriver leftDriver;
    yarp::dev::PolyDriver rightDriver;
    ASSERT_TRUE(openClient(leftDriver, "/StereoDepthCameraTest/left", "/stereodepthcamera/left"));
    ASSERT_TRUE(openClient(rightDriver, "/StereoDepthCameraTest/right", "/stereodepthcamera/right"));

    yarp::dev::IRGBDSensor* leftSensor = nullptr;
    yarp::dev::IRGBDSensor* rightSensor = nullptr;
    ASSERT_TRUE(leftDriver.view(leftSensor));
    ASSERT_TRUE(rightDriver.view(rightSensor));
    ASSERT_NE(leftSensor, nullptr);
    ASSERT_NE(rightSensor, nullptr);

    yarp::os::BufferedPort<yarp::sig::FlexImage> stereoPort;
    ASSERT_TRUE(stereoPort.open("/StereoDepthCameraTest/stereo:i"));
    ASSERT_TRUE(yarp::os::Network::connect("/stereodepthcamera/rgbImage:o",
                                          "/StereoDepthCameraTest/stereo:i"));

    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::sig::FlexImage leftRgb;
    yarp::sig::FlexImage rightRgb;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> leftDepth;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> rightDepth;
    ASSERT_TRUE(readImages(*leftSensor, leftRgb, leftDepth));
    ASSERT_TRUE(readImages(*rightSensor, rightRgb, rightDepth));

    EXPECT_EQ(leftRgb.width(), 320);
    EXPECT_EQ(leftRgb.height(), 240);
    EXPECT_EQ(rightRgb.width(), 320);
    EXPECT_EQ(rightRgb.height(), 240);
    EXPECT_EQ(leftDepth.width(), 320);
    EXPECT_EQ(leftDepth.height(), 240);
    EXPECT_EQ(rightDepth.width(), 320);
    EXPECT_EQ(rightDepth.height(), 240);

    yarp::sig::FlexImage* stereoRgb = nullptr;
    for (std::size_t attempt = 0; attempt < 20 && stereoRgb == nullptr; ++attempt) {
        fixture.Server()->Run(/*_blocking=*/true, 100, /*_paused=*/false);
        stereoRgb = stereoPort.read(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    ASSERT_NE(stereoRgb, nullptr);
    ASSERT_EQ(stereoRgb->width(), leftRgb.width() + rightRgb.width());
    ASSERT_EQ(stereoRgb->height(), leftRgb.height());
    ASSERT_EQ(stereoRgb->getPixelSize(), leftRgb.getPixelSize());

    const int sampleX = 10;
    const int sampleY = 10;
    const auto* leftPixel = leftRgb.getPixelAddress(sampleX, sampleY);
    const auto* rightPixel = rightRgb.getPixelAddress(sampleX, sampleY);
    const auto* stereoLeftPixel = stereoRgb->getPixelAddress(sampleX, sampleY);
    const auto* stereoRightPixel = stereoRgb->getPixelAddress(leftRgb.width() + sampleX, sampleY);
    for (std::size_t channel = 0; channel < leftRgb.getPixelSize(); ++channel) {
        EXPECT_EQ(stereoLeftPixel[channel], leftPixel[channel]);
        EXPECT_EQ(stereoRightPixel[channel], rightPixel[channel]);
    }

    EXPECT_EQ(leftSensor->getRgbWidth(), 320);
    EXPECT_EQ(rightSensor->getRgbWidth(), 320);
    EXPECT_EQ(leftSensor->getDepthWidth(), 320);
    EXPECT_EQ(rightSensor->getDepthWidth(), 320);

    fixture.Server()->Stop();
}
