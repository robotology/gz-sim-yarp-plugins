#include <gtest/gtest.h>
#include <gz/sim/TestFixture.hh>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>

TEST(ImuTest, PluginTest)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);

    // Instantiate test fixture
    gz::sim::TestFixture fixture("../../../tests/imu/model.sdf");

    int iterations = 1000;
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::os::Property option;
    option.put("device", "multipleanalogsensorsclient");
    option.put("remote", "/IMU");
    option.put("timeout", 1.0);
    option.put("local", "/ImuTest");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));

    yarp::dev::IThreeAxisGyroscopes* igyroscope = nullptr;
    yarp::dev::IOrientationSensors* iorientation = nullptr;
    yarp::dev::IThreeAxisLinearAccelerometers* iaccelerometer = nullptr;

    ASSERT_TRUE(driver.view(igyroscope));
    ASSERT_TRUE(driver.view(iorientation));
    ASSERT_TRUE(driver.view(iaccelerometer));

    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    yarp::sig::Vector measureGyroscope(3);
    yarp::sig::Vector measureOrientation(3);
    yarp::sig::Vector measureAccelerometer(3);
    double timestampGyroscope;
    double timestampOrientation;
    double timestampAccelerometer;
    yarp::dev::MAS_status statusGyroscope;
    yarp::dev::MAS_status statusOrientation;
    yarp::dev::MAS_status statusAccelerometer;

    size_t maxNrOfReadingAttempts = 20;
    bool readSuccessful = false;

    for (size_t i = 0; (i < maxNrOfReadingAttempts) && !readSuccessful; i++)
    {
        readSuccessful
            = igyroscope->getThreeAxisGyroscopeMeasure(0, measureGyroscope, timestampGyroscope)
              && iorientation->getOrientationSensorMeasureAsRollPitchYaw(0,
                                                                         measureOrientation,
                                                                         timestampOrientation)
              && iaccelerometer->getThreeAxisLinearAccelerometerMeasure(0,
                                                                        measureAccelerometer,
                                                                        timestampAccelerometer);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ASSERT_TRUE(readSuccessful);

    statusGyroscope = igyroscope->getThreeAxisGyroscopeStatus(0);
    ASSERT_EQ(statusGyroscope, yarp::dev::MAS_OK);
    statusOrientation = iorientation->getOrientationSensorStatus(0);
    ASSERT_EQ(statusOrientation, yarp::dev::MAS_OK);
    statusAccelerometer = iaccelerometer->getThreeAxisLinearAccelerometerStatus(0);
    ASSERT_EQ(statusAccelerometer, yarp::dev::MAS_OK);

    EXPECT_NEAR(measureGyroscope(0), 0.0, 1e-2);
    EXPECT_NEAR(measureGyroscope(1), 0.0, 1e-2);
    EXPECT_NEAR(measureGyroscope(2), 0.0, 1e-2);
    EXPECT_GT(timestampGyroscope, 0.0);

    EXPECT_NEAR(measureOrientation(0), 0.0, 1e-2);
    EXPECT_NEAR(measureOrientation(1), 0.0, 1e-2);
    EXPECT_NEAR(measureOrientation(2), 1.0, 1e-2);
    EXPECT_GT(timestampOrientation, 0.0);

    EXPECT_NEAR(measureAccelerometer(0), 0.0, 1e-2);
    EXPECT_NEAR(measureAccelerometer(1), 0.0, 1e-2);
    EXPECT_NEAR(measureAccelerometer(2), 9.8, 1e-2);
    EXPECT_GT(timestampAccelerometer, 0.0);
}
