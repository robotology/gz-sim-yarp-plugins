#include <chrono>
#include <cmath>
#include <cstddef>
#include <thread>

#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/sim/TestFixture.hh>

#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

TEST(LaserTest, PluginTest)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    gz::sim::TestFixture fixture("../../../tests/laser/model.sdf");

    int iterations = 1000;
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "Rangefinder2DClient");
    option.put("remote", "/laser");
    option.put("local", "/LaserTest");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));
    yarp::dev::IRangefinder2D* irange = nullptr;
    ASSERT_TRUE(driver.view(irange));

    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::sig::Vector measure(360);
    double timestamp;

    size_t maxNrOfReadingAttempts = 20;
    bool readSuccessful = false;
    for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
    {
        readSuccessful = irange->getRawData(measure, &timestamp);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(readSuccessful);

    double distance = 4.0;
    double size_box = 1.0;
    EXPECT_NEAR(measure(0), distance - size_box / 2, 1e-1);
    EXPECT_NEAR(measure(1), distance - size_box / 2, 1e-1);
    EXPECT_NEAR(measure(2), distance - size_box / 2, 1e-1);
    EXPECT_EQ(measure(100), INFINITY);
    EXPECT_EQ(measure(200), INFINITY);
    EXPECT_EQ(measure(300), INFINITY);
    EXPECT_EQ(measure(359), INFINITY);
    EXPECT_GT(timestamp, 0.0);
}
