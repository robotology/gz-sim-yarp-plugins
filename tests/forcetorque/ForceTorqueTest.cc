#include <chrono>
#include <cstddef>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/sim/TestFixture.hh>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

TEST(ForceTorqueTest, PluginTest)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    gz::sim::TestFixture fixture("../../../tests/forcetorque/model.sdf");

    int iterations = 1000;
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "multipleanalogsensorsclient");
    option.put("remote", "/forcetorque");
    option.put("timeout", 1.0);
    option.put("local", "/ForceTorqueTest");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));

    yarp::dev::ISixAxisForceTorqueSensors* isixaxis = nullptr;

    ASSERT_TRUE(driver.view(isixaxis));

    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);
    yarp::sig::Vector measure(6);
    std::string sensorName;
    double timestamp;

    isixaxis->getSixAxisForceTorqueSensorName(0, sensorName);
    size_t maxNrOfReadingAttempts = 20;
    bool readSuccessful = false;
    for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
    {
        readSuccessful = isixaxis->getSixAxisForceTorqueSensorMeasure(0, measure, timestamp);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(readSuccessful);

    yarp::dev::MAS_status status;
    status = isixaxis->getSixAxisForceTorqueSensorStatus(0);
    ASSERT_EQ(status, yarp::dev::MAS_OK);

    EXPECT_NEAR(measure(0), 0.0, 1e-2);
    EXPECT_NEAR(measure(1), 0.0, 1e-2);
    EXPECT_NEAR(measure(2), -9.8 * 10, 1e-2);
    EXPECT_NEAR(measure(3), 0.0, 1e-2);
    EXPECT_NEAR(measure(4), 0.0, 1e-2);
    EXPECT_NEAR(measure(5), 0.0, 1e-2);
    EXPECT_GT(timestamp, 0.0);
}
