#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/TestFixture.hh>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>


        
const double depth_tolerance = 0.001; // 1 mm
int blue[] = {0, 0, 255};
int dark_blue[] = {0, 0, 100};
int grey[] = {218, 218, 218};
int dark_grey[] = {175, 175, 175};

TEST(DepthCameraTest, PluginTest)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model.sdf";
    gz::sim::TestFixture fixture(modelPath.string());
    fixture.Finalize();

    int iterations = 1000;

    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "RGBDSensorClient");
    option.put("localImagePort", "/RGBD_nwc/Image:o");
    option.put("localDepthPort", "/RGBD_nwc/Depth:o");
    option.put("remoteImagePort", "/depthcamera/rgbImage:o");
    option.put("remoteDepthPort", "/depthcamera/depthImage:o");
    option.put("localRpcPort", "/RGBD_nwc/rpc:o");
    option.put("remoteRpcPort", "/depthcamera/rpc:i");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));

    yarp::dev::IRGBDSensor* irgbdsensor = nullptr;

    ASSERT_TRUE(driver.view(irgbdsensor));
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);
    {
        yarp::sig::FlexImage image;
        size_t maxNrOfReadingAttempts = 20;
        bool readSuccessful = false;
        for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
        {
            yarp::os::Stamp stamp;
            readSuccessful = irgbdsensor->getRgbImage(image, &stamp);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        ASSERT_TRUE(readSuccessful);

        EXPECT_EQ(image.width(), 640);
        EXPECT_EQ(image.height(), 480);

        // background - grey
        unsigned char* pixel = image.getPixelAddress(0, 40);
        EXPECT_EQ(int(pixel[0]), grey[0]);
        EXPECT_EQ(int(pixel[1]), grey[1]);
        EXPECT_EQ(int(pixel[2]), grey[2]);

        // sphere - dark blue
        pixel = image.getPixelAddress(152, 204);
        EXPECT_EQ(int(pixel[0]), dark_blue[0]);
        EXPECT_EQ(int(pixel[1]), dark_blue[1]);
        EXPECT_EQ(int(pixel[2]), dark_blue[2]);

        // ground plane - dark grey
        pixel = image.getPixelAddress(151, 289);
        EXPECT_EQ(int(pixel[0]), dark_grey[0]);
        EXPECT_EQ(int(pixel[1]), dark_grey[1]);
        EXPECT_EQ(int(pixel[2]), dark_grey[2]);
    }

    {
        yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;

        size_t maxNrOfReadingAttempts = 20;
        bool readSuccessful = false;
        for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
        {
            yarp::os::Stamp stamp;
            readSuccessful = irgbdsensor->getDepthImage(depthImage, &stamp);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        ASSERT_TRUE(readSuccessful);

        EXPECT_EQ(depthImage.width(), 640);
        EXPECT_EQ(depthImage.height(), 480);
   
        // background 
        EXPECT_TRUE(std::isinf(depthImage.pixel(0, 40)));
        // sphere
        EXPECT_NEAR(depthImage.pixel(152, 204), 4.531866, depth_tolerance); // sphere distance in meters
        // ground plane
        EXPECT_NEAR(depthImage.pixel(151, 289), 1.11972, depth_tolerance); // ground plane distance in meters

    }


    {
        yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;
        yarp::sig::FlexImage image;
        size_t maxNrOfReadingAttempts = 20;
        bool readSuccessful = false;
        for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
        {
            yarp::os::Stamp stamp;
            readSuccessful = irgbdsensor->getImages(image, depthImage, &stamp, &stamp);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        ASSERT_TRUE(readSuccessful);

        EXPECT_EQ(depthImage.width(), 640);
        EXPECT_EQ(depthImage.height(), 480);

        // background
        EXPECT_TRUE(std::isinf(depthImage.pixel(0, 40)));
        unsigned char* pixel = image.getPixelAddress(0, 40);
        EXPECT_EQ(int(pixel[0]), grey[0]);
        EXPECT_EQ(int(pixel[1]), grey[1]);
        EXPECT_EQ(int(pixel[2]), grey[2]);
        // sphere
        EXPECT_NEAR(depthImage.pixel(152, 204), 4.531866, depth_tolerance); // sphere distance in meters
        pixel = image.getPixelAddress(152, 204);
        EXPECT_EQ(int(pixel[0]), dark_blue[0]);
        EXPECT_EQ(int(pixel[1]), dark_blue[1]);
        EXPECT_EQ(int(pixel[2]), dark_blue[2]);
        // ground plane
        EXPECT_NEAR(depthImage.pixel(151, 289), 1.11972, depth_tolerance); // ground plane distance in meters
        pixel = image.getPixelAddress(151, 289);
        EXPECT_EQ(int(pixel[0]), dark_grey[0]);
        EXPECT_EQ(int(pixel[1]), dark_grey[1]);
        EXPECT_EQ(int(pixel[2]), dark_grey[2]);
    }

    fixture.Server()->Stop();
}

TEST(DepthCameraTest, Functions)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model.sdf";
    gz::sim::TestFixture fixture(modelPath.string());
    fixture.Finalize();

    int iterations = 1000;

    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "RGBDSensorClient");
    option.put("localImagePort", "/RGBD_nwc/Image:o");
    option.put("localDepthPort", "/RGBD_nwc/Depth:o");
    option.put("remoteImagePort", "/depthcamera/rgbImage:o");
    option.put("remoteDepthPort", "/depthcamera/depthImage:o");
    option.put("localRpcPort", "/RGBD_nwc/rpc:o");
    option.put("remoteRpcPort", "/depthcamera/rpc:i");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));

    yarp::dev::IRGBDSensor* irgbdsensor = nullptr;

    ASSERT_TRUE(driver.view(irgbdsensor));
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    EXPECT_EQ(irgbdsensor->getRgbHeight(), 480);
    EXPECT_EQ(irgbdsensor->getRgbWidth(),  640);
    EXPECT_FALSE(irgbdsensor->setRgbResolution(640, 480));
    double horizontalFov, verticalFov;
    EXPECT_TRUE(irgbdsensor->getRgbFOV(horizontalFov, verticalFov));
    EXPECT_EQ(horizontalFov, 1.5708);
    EXPECT_EQ(verticalFov, 0.0);
    // EXPECT_FALSE(irgbdsensor->setRgbFOV(horizontalFov, verticalFov)); // Broken on RGBDSensorClient side https://github.com/robotology/yarp/pull/3190
    bool mirror;
    EXPECT_TRUE(irgbdsensor->getRgbMirroring(mirror));
    EXPECT_FALSE(irgbdsensor->setRgbMirroring(mirror));
    yarp::os::Property intrinsic;
    EXPECT_TRUE(irgbdsensor->getRgbIntrinsicParam(intrinsic));
    EXPECT_EQ(intrinsic.find("physFocalLength").asFloat64(), 0.0);
    EXPECT_EQ(intrinsic.find("focalLengthX").asFloat64(), 554.256);
    EXPECT_EQ(intrinsic.find("focalLengthY").asFloat64(), 554.256);
    EXPECT_EQ(intrinsic.find("k1").asFloat64(), 1.0);
    EXPECT_EQ(intrinsic.find("k2").asFloat64(), 2.0);
    EXPECT_EQ(intrinsic.find("k3").asFloat64(), 3.0);
    EXPECT_EQ(intrinsic.find("t1").asFloat64(), 4.0);
    EXPECT_EQ(intrinsic.find("t2").asFloat64(), 5.0);
    EXPECT_EQ(intrinsic.find("principalPointX").asFloat64(), 322);
    EXPECT_EQ(intrinsic.find("principalPointY").asFloat64(), 243);
    EXPECT_EQ(intrinsic.find("rectificationMatrix").asList()->toString(), "1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0");
    EXPECT_EQ(intrinsic.find("distortionModel").asString(), "plumb_bob");
    EXPECT_EQ(irgbdsensor->getDepthHeight(), 480);
    EXPECT_EQ(irgbdsensor->getDepthWidth(), 640);
    EXPECT_FALSE(irgbdsensor->setDepthResolution(640, 480));
    EXPECT_TRUE(irgbdsensor->getDepthFOV(horizontalFov, verticalFov));
    EXPECT_FALSE(irgbdsensor->setDepthFOV(horizontalFov, verticalFov));
    EXPECT_TRUE(irgbdsensor->getDepthIntrinsicParam(intrinsic));
    EXPECT_EQ(irgbdsensor->getDepthAccuracy(), 0.00001);
    double nearPlane, farPlane;
    EXPECT_TRUE(irgbdsensor->getDepthClipPlanes(nearPlane, farPlane));
    EXPECT_EQ(nearPlane, 0.1);
    EXPECT_EQ(farPlane, 100.0);
    EXPECT_FALSE(irgbdsensor->setDepthAccuracy(0.00001));
    EXPECT_FALSE(irgbdsensor->setDepthClipPlanes(nearPlane, farPlane));
    EXPECT_TRUE(irgbdsensor->getDepthMirroring(mirror));
    EXPECT_FALSE(irgbdsensor->setDepthMirroring(mirror));
    // yarp::sig::Matrix extrinsic; 
    // EXPECT_TRUE(irgbdsensor->getExtrinsicParam(extrinsic)); // Probably broken too see https://github.com/robotology/yarp/blob/b9c4888aef50b2014221d53ce8ee9c45175a74dd/src/devices/networkWrappers/RGBDSensorClient/RGBDSensorClient.cpp#L159
    // yDebug() << extrinsic.rows() << extrinsic.cols();
    // EXPECT_EQ(extrinsic.rows(), 4);
    // EXPECT_EQ(extrinsic.cols(), 4);
    // EXPECT_EQ(extrinsic[0][0], 1);
    // EXPECT_EQ(extrinsic[1][1], 1);
    // EXPECT_EQ(extrinsic[2][2], 1);
    // EXPECT_EQ(extrinsic[3][3], 1);
    fixture.Server()->Stop();

}
