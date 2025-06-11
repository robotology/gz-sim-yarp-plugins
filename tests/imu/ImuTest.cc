#include <DeviceRegistry.hh>
#include <ImuDriver.cpp>
#include <TestHelpers.hh>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/TestFixture.hh>
#include <sdf/Element.hh>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

class ImuFixture : public testing::Test
{
protected:
    ImuFixture()
        : testFixture{(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model.sdf").string()}
    {
        gz::common::Console::SetVerbosity(4);

        testFixture.OnConfigure([&](const gz::sim::Entity& _worldEntity,
                                    const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                                    gz::sim::EntityComponentManager& _ecm,
                                    gz::sim::EventManager& /*_eventMgr*/) {
            auto imuDrivers
                = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::gzyarp::ImuDriver>(
                    _ecm);
            ASSERT_TRUE(imuDrivers.size() == 1);
            driver = imuDrivers[0];
            ASSERT_TRUE(driver != nullptr);
            ASSERT_TRUE(driver->view(igyroscope));
            ASSERT_TRUE(driver->view(iorientation));
            ASSERT_TRUE(driver->view(iaccelerometer));

            configured = true;
        });
    }

    gz::sim::TestFixture testFixture;
    std::string deviceScopedName = "model/sensor_box/link/link_1/sensor/imu_sensor/"
                                   "imu_plugin_device";
    yarp::dev::gzyarp::ImuDriver* driver;
    yarp::dev::IThreeAxisGyroscopes* igyroscope = nullptr;
    yarp::dev::IOrientationSensors* iorientation = nullptr;
    yarp::dev::IThreeAxisLinearAccelerometers* iaccelerometer = nullptr;
    bool configured{false};
};

TEST_F(ImuFixture, ImuTest)
{
    int iterations = 1000;

    testFixture.Finalize();

    testFixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    testFixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    // option.put("device", "multipleanalogsensorsclient");

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

    ASSERT_TRUE(configured);
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
    EXPECT_NEAR(measureOrientation(2), 0.0, 1e-2);
    EXPECT_GT(timestampOrientation, 0.0);

    EXPECT_NEAR(measureAccelerometer(0), 0.0, 1e-2);
    EXPECT_NEAR(measureAccelerometer(1), 0.0, 1e-2);
    EXPECT_NEAR(measureAccelerometer(2), 9.8, 1e-2);
    EXPECT_GT(timestampAccelerometer, 0.0);
}
