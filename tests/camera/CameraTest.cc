#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/TestFixture.hh>

#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Image.h>

TEST(CameraTest, PluginTest)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model.sdf";
    gz::sim::TestFixture fixture(modelPath.string());

    int iterations = 1000;

    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "frameGrabber_nwc_yarp");
    option.put("remote", "/camera");
    option.put("local", "/CameraTest");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));

    yarp::dev::IFrameGrabberImage* igrabimg = nullptr;

    ASSERT_TRUE(driver.view(igrabimg));
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> image;

    size_t maxNrOfReadingAttempts = 20;
    bool readSuccessful = false;
    for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
    {
        readSuccessful = igrabimg->getImage(image);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(readSuccessful);

    int blue[] = {0, 0, 255};
    int dark_blue[] = {0, 0, 100};
    int grey[] = {218, 218, 218};
    int dark_grey[] = {162, 162, 162};

    // background - grey
    unsigned char* pixel = image.getPixelAddress(0, 40);
    EXPECT_EQ(int(pixel[0]), grey[0]);
    EXPECT_EQ(int(pixel[1]), grey[1]);
    EXPECT_EQ(int(pixel[2]), grey[2]);

    // timestamp - blue
    pixel = image.getPixelAddress(35, 22);
    EXPECT_EQ(int(pixel[0]), blue[0]);
    EXPECT_EQ(int(pixel[1]), blue[1]);
    EXPECT_EQ(int(pixel[2]), blue[2]);

    // sphere - dark blue
    pixel = image.getPixelAddress(218, 221);
    EXPECT_EQ(int(pixel[0]), dark_blue[0]);
    EXPECT_EQ(int(pixel[1]), dark_blue[1]);
    EXPECT_EQ(int(pixel[2]), dark_blue[2]);

    // ground plane - dark grey
    pixel = image.getPixelAddress(289, 246);
    EXPECT_EQ(int(pixel[0]), dark_grey[0]);
    EXPECT_EQ(int(pixel[1]), dark_grey[1]);
    EXPECT_EQ(int(pixel[2]), dark_grey[2]);
}

TEST(CameraTest, HorizontalFlip)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model_hor_flip.sdf";
    gz::sim::TestFixture fixture(modelPath.string());

    int iterations = 1000;
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "frameGrabber_nwc_yarp");
    option.put("remote", "/camera");
    option.put("local", "/CameraTest");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));

    yarp::dev::IFrameGrabberImage* igrabimg = nullptr;

    ASSERT_TRUE(driver.view(igrabimg));
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> image;

    size_t maxNrOfReadingAttempts = 20;
    bool readSuccessful = false;
    for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
    {
        readSuccessful = igrabimg->getImage(image);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(readSuccessful);

    int blue[] = {0, 0, 255};
    int dark_blue[] = {0, 0, 100};
    int grey[] = {218, 218, 218};
    int dark_grey[] = {162, 162, 162};

    // background - grey
    unsigned char* pixel = image.getPixelAddress(0, 40);
    EXPECT_EQ(int(pixel[0]), grey[0]);
    EXPECT_EQ(int(pixel[1]), grey[1]);
    EXPECT_EQ(int(pixel[2]), grey[2]);

    // timestamp - blue
    pixel = image.getPixelAddress(35, 22);
    EXPECT_EQ(int(pixel[0]), blue[0]);
    EXPECT_EQ(int(pixel[1]), blue[1]);
    EXPECT_EQ(int(pixel[2]), blue[2]);

    // sphere - dark blue
    pixel = image.getPixelAddress(420, 220);
    EXPECT_EQ(int(pixel[0]), dark_blue[0]);
    EXPECT_EQ(int(pixel[1]), dark_blue[1]);
    EXPECT_EQ(int(pixel[2]), dark_blue[2]);

    // ground plane - dark grey
    pixel = image.getPixelAddress(304, 247);
    EXPECT_EQ(int(pixel[0]), dark_grey[0]);
    EXPECT_EQ(int(pixel[1]), dark_grey[1]);
    EXPECT_EQ(int(pixel[2]), dark_grey[2]);
}

TEST(CameraTest, VerticalFlip)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model_ver_flip.sdf";
    gz::sim::TestFixture fixture(modelPath.string());

    int iterations = 1000;
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "frameGrabber_nwc_yarp");
    option.put("remote", "/camera");
    option.put("local", "/CameraTest");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));

    yarp::dev::IFrameGrabberImage* igrabimg = nullptr;

    ASSERT_TRUE(driver.view(igrabimg));
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> image;

    size_t maxNrOfReadingAttempts = 20;
    bool readSuccessful = false;
    for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
    {
        readSuccessful = igrabimg->getImage(image);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(readSuccessful);

    int blue[] = {0, 0, 255};
    int dark_blue[] = {0, 0, 100};
    int grey[] = {218, 218, 218};
    int dark_grey[] = {162, 162, 162};

    // background - grey
    unsigned char* pixel = image.getPixelAddress(270, 400);
    EXPECT_EQ(int(pixel[0]), grey[0]);
    EXPECT_EQ(int(pixel[1]), grey[1]);
    EXPECT_EQ(int(pixel[2]), grey[2]);

    // timestamp - blue
    pixel = image.getPixelAddress(35, 22);
    EXPECT_EQ(int(pixel[0]), blue[0]);
    EXPECT_EQ(int(pixel[1]), blue[1]);
    EXPECT_EQ(int(pixel[2]), blue[2]);

    // sphere - dark blue
    pixel = image.getPixelAddress(219, 254);
    EXPECT_EQ(int(pixel[0]), dark_blue[0]);
    EXPECT_EQ(int(pixel[1]), dark_blue[1]);
    EXPECT_EQ(int(pixel[2]), dark_blue[2]);

    // ground plane - dark grey
    pixel = image.getPixelAddress(380, 232);
    EXPECT_EQ(int(pixel[0]), dark_grey[0]);
    EXPECT_EQ(int(pixel[1]), dark_grey[1]);
    EXPECT_EQ(int(pixel[2]), dark_grey[2]);
}

TEST(CameraTest, HorizontalVerticalFlip)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model_hor_ver_flip.sdf";
    gz::sim::TestFixture fixture(modelPath.string());

    int iterations = 1000;
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "frameGrabber_nwc_yarp");
    option.put("remote", "/camera");
    option.put("local", "/CameraTest");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));

    yarp::dev::IFrameGrabberImage* igrabimg = nullptr;

    ASSERT_TRUE(driver.view(igrabimg));
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> image;

    size_t maxNrOfReadingAttempts = 20;
    bool readSuccessful = false;
    for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
    {
        readSuccessful = igrabimg->getImage(image);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(readSuccessful);

    int blue[] = {0, 0, 255};
    int dark_blue[] = {0, 0, 100};
    int grey[] = {218, 218, 218};
    int dark_grey[] = {162, 162, 162};

    // background - grey
    unsigned char* pixel = image.getPixelAddress(301, 389);
    EXPECT_EQ(int(pixel[0]), grey[0]);
    EXPECT_EQ(int(pixel[1]), grey[1]);
    EXPECT_EQ(int(pixel[2]), grey[2]);

    // timestamp - blue
    pixel = image.getPixelAddress(35, 22);
    EXPECT_EQ(int(pixel[0]), blue[0]);
    EXPECT_EQ(int(pixel[1]), blue[1]);
    EXPECT_EQ(int(pixel[2]), blue[2]);

    // sphere - dark blue
    pixel = image.getPixelAddress(416, 259);
    EXPECT_EQ(int(pixel[0]), dark_blue[0]);
    EXPECT_EQ(int(pixel[1]), dark_blue[1]);
    EXPECT_EQ(int(pixel[2]), dark_blue[2]);

    // ground plane - dark grey
    pixel = image.getPixelAddress(334, 233);
    EXPECT_EQ(int(pixel[0]), dark_grey[0]);
    EXPECT_EQ(int(pixel[1]), dark_grey[1]);
    EXPECT_EQ(int(pixel[2]), dark_grey[2]);
}
