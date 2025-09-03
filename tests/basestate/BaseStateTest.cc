#include <BaseStateDriver.cpp>
#include <DeviceRegistry.hh>
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

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

class BaseStateFixture : public testing::Test
{
protected:
    BaseStateFixture()
        : testFixture{(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model.sdf").string()}
    {
        testFixture.OnConfigure([&](const gz::sim::Entity& _worldEntity,
                                    const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                                    gz::sim::EntityComponentManager& _ecm,
                                    gz::sim::EventManager& /*_eventMgr*/) {
            auto baseStateDrivers
                = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::gzyarp::BaseStateDriver>(
                    _ecm);
            ASSERT_TRUE(baseStateDrivers.size() == 1);
            baseStateDriver = baseStateDrivers[0];
            ASSERT_TRUE(baseStateDriver != nullptr);

            // Test various interface views
            ASSERT_TRUE(baseStateDriver->view(iAnalog));
            ASSERT_TRUE(baseStateDriver->view(iPreciselyTimed));
            ASSERT_TRUE(baseStateDriver->view(iPositionSensors));
            ASSERT_TRUE(baseStateDriver->view(iOrientationSensors));
            ASSERT_TRUE(baseStateDriver->view(iLinearVelocitySensors));
            ASSERT_TRUE(baseStateDriver->view(iThreeAxisGyroscopes));
            ASSERT_TRUE(baseStateDriver->view(iThreeAxisLinearAccelerometers));
            ASSERT_TRUE(baseStateDriver->view(iThreeAxisAngularAccelerometers));

            configured = true;
        });
    }

    void SetUp() override
    {
        yarp::os::NetworkBase::setLocalMode(true);
        gz::common::Console::SetVerbosity(4);
        testFixture.Finalize();
    }

    void TearDown() override
    {
        testFixture.Server()->Stop();
    }

    // Helper function to run simulation iterations and wait for data
    void runSimulationAndWaitForData(int iterations = 1000)
    {
        testFixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);
    }

    gz::sim::TestFixture testFixture;
    yarp::dev::gzyarp::BaseStateDriver* baseStateDriver{nullptr};

    // Interface pointers
    yarp::dev::IAnalogSensor* iAnalog{nullptr};
    yarp::dev::IPreciselyTimed* iPreciselyTimed{nullptr};
    yarp::dev::IPositionSensors* iPositionSensors{nullptr};
    yarp::dev::IOrientationSensors* iOrientationSensors{nullptr};
    yarp::dev::ILinearVelocitySensors* iLinearVelocitySensors{nullptr};
    yarp::dev::IThreeAxisGyroscopes* iThreeAxisGyroscopes{nullptr};
    yarp::dev::IThreeAxisLinearAccelerometers* iThreeAxisLinearAccelerometers{nullptr};
    yarp::dev::IThreeAxisAngularAccelerometers* iThreeAxisAngularAccelerometers{nullptr};

    bool configured{false};
};

TEST_F(BaseStateFixture, CheckAnalogSensorInterface)
{
    runSimulationAndWaitForData(1000);
    ASSERT_TRUE(configured);

    // Test IAnalogSensor interface
    ASSERT_EQ(iAnalog->getChannels(), 18); // BaseState has 18 channels
    ASSERT_EQ(iAnalog->getState(0), yarp::dev::IAnalogSensor::AS_OK);

    yarp::sig::Vector measures;
    ASSERT_EQ(iAnalog->read(measures), yarp::dev::IAnalogSensor::AS_OK);
    ASSERT_EQ(measures.size(), 18);

    // Verify all values are reasonable (not NaN or infinite)
    for (size_t i = 0; i < measures.size(); i++)
    {
        ASSERT_TRUE(std::isfinite(measures[i]))
            << "Channel " << i << " has invalid value: " << measures[i];
    }
}

TEST_F(BaseStateFixture, CheckPositionSensors)
{
    runSimulationAndWaitForData(1000);
    ASSERT_TRUE(configured);

    // Test IPositionSensors interface
    ASSERT_EQ(iPositionSensors->getNrOfPositionSensors(), 1);

    std::string sensorName;
    ASSERT_TRUE(iPositionSensors->getPositionSensorName(0, sensorName));
    ASSERT_FALSE(sensorName.empty());

    ASSERT_EQ(iPositionSensors->getPositionSensorStatus(0), yarp::dev::MAS_OK);

    yarp::sig::Vector position(3);
    double timestamp;
    ASSERT_TRUE(iPositionSensors->getPositionSensorMeasure(0, position, timestamp));

    ASSERT_EQ(position.size(), 3);
    ASSERT_GT(timestamp, 0.0);

    // Verify position values are finite
    for (size_t i = 0; i < position.size(); i++)
    {
        ASSERT_TRUE(std::isfinite(position[i]))
            << "Position component " << i << " has invalid value: " << position[i];
    }

    // The model should be in contact with the ground after the simulation ran, hence the base
    // position should be at the center of the box
    EXPECT_NEAR(position[2], 0.05, 0.01);
}

TEST_F(BaseStateFixture, CheckOrientationSensors)
{
    runSimulationAndWaitForData();
    ASSERT_TRUE(configured);

    // Test IOrientationSensors interface
    ASSERT_EQ(iOrientationSensors->getNrOfOrientationSensors(), 1);

    std::string sensorName;
    ASSERT_TRUE(iOrientationSensors->getOrientationSensorName(0, sensorName));
    ASSERT_FALSE(sensorName.empty());

    ASSERT_EQ(iOrientationSensors->getOrientationSensorStatus(0), yarp::dev::MAS_OK);

    yarp::sig::Vector orientation(3);
    double timestamp;
    ASSERT_TRUE(
        iOrientationSensors->getOrientationSensorMeasureAsRollPitchYaw(0, orientation, timestamp));

    ASSERT_EQ(orientation.size(), 3);
    ASSERT_GT(timestamp, 0.0);

    // Verify orientation values are finite
    for (size_t i = 0; i < orientation.size(); i++)
    {
        ASSERT_TRUE(std::isfinite(orientation[i]))
            << "Orientation component " << i << " has invalid value: " << orientation[i];
    }

    // Initial orientation should be close to zero (no rotation)
    EXPECT_NEAR(orientation[0], 0.0, 0.1); // roll
    EXPECT_NEAR(orientation[1], 0.0, 0.1); // pitch
    EXPECT_NEAR(orientation[2], 0.0, 0.1); // yaw
}

TEST_F(BaseStateFixture, CheckLinearVelocitySensors)
{
    runSimulationAndWaitForData();
    ASSERT_TRUE(configured);

    // Test ILinearVelocitySensors interface
    ASSERT_EQ(iLinearVelocitySensors->getNrOfLinearVelocitySensors(), 1);

    std::string sensorName;
    ASSERT_TRUE(iLinearVelocitySensors->getLinearVelocitySensorName(0, sensorName));
    ASSERT_FALSE(sensorName.empty());

    ASSERT_EQ(iLinearVelocitySensors->getLinearVelocitySensorStatus(0), yarp::dev::MAS_OK);

    yarp::sig::Vector velocity(3);
    double timestamp;
    ASSERT_TRUE(iLinearVelocitySensors->getLinearVelocitySensorMeasure(0, velocity, timestamp));

    ASSERT_EQ(velocity.size(), 3);
    ASSERT_GT(timestamp, 0.0);

    // Verify velocity values are finite
    for (size_t i = 0; i < velocity.size(); i++)
    {
        ASSERT_TRUE(std::isfinite(velocity[i]))
            << "Velocity component " << i << " has invalid value: " << velocity[i];
    }

    // Initially the object should be stationary (or settling from initial drop)
    // We expect small velocities
    EXPECT_LT(std::abs(velocity[0]), 1.0); // vx
    EXPECT_LT(std::abs(velocity[1]), 1.0); // vy
    // vz might be non-zero due to gravity/settling
}

TEST_F(BaseStateFixture, CheckThreeAxisGyroscopes)
{
    runSimulationAndWaitForData();
    ASSERT_TRUE(configured);

    // Test IThreeAxisGyroscopes interface
    ASSERT_EQ(iThreeAxisGyroscopes->getNrOfThreeAxisGyroscopes(), 1);

    std::string sensorName;
    ASSERT_TRUE(iThreeAxisGyroscopes->getThreeAxisGyroscopeName(0, sensorName));
    ASSERT_FALSE(sensorName.empty());

    ASSERT_EQ(iThreeAxisGyroscopes->getThreeAxisGyroscopeStatus(0), yarp::dev::MAS_OK);

    yarp::sig::Vector angularVelocity(3);
    double timestamp;
    ASSERT_TRUE(iThreeAxisGyroscopes->getThreeAxisGyroscopeMeasure(0, angularVelocity, timestamp));

    ASSERT_EQ(angularVelocity.size(), 3);
    ASSERT_GT(timestamp, 0.0);

    // Verify angular velocity values are finite
    for (size_t i = 0; i < angularVelocity.size(); i++)
    {
        ASSERT_TRUE(std::isfinite(angularVelocity[i]))
            << "Angular velocity component " << i << " has invalid value: " << angularVelocity[i];
    }

    // Initially the object should have small angular velocities
    EXPECT_LT(std::abs(angularVelocity[0]), 1.0); // wx
    EXPECT_LT(std::abs(angularVelocity[1]), 1.0); // wy
    EXPECT_LT(std::abs(angularVelocity[2]), 1.0); // wz
}

TEST_F(BaseStateFixture, CheckThreeAxisLinearAccelerometers)
{
    runSimulationAndWaitForData();
    ASSERT_TRUE(configured);

    // Test IThreeAxisLinearAccelerometers interface
    ASSERT_EQ(iThreeAxisLinearAccelerometers->getNrOfThreeAxisLinearAccelerometers(), 1);

    std::string sensorName;
    ASSERT_TRUE(iThreeAxisLinearAccelerometers->getThreeAxisLinearAccelerometerName(0, sensorName));
    ASSERT_FALSE(sensorName.empty());

    ASSERT_EQ(iThreeAxisLinearAccelerometers->getThreeAxisLinearAccelerometerStatus(0),
              yarp::dev::MAS_OK);

    yarp::sig::Vector acceleration(3);
    double timestamp;
    ASSERT_TRUE(iThreeAxisLinearAccelerometers->getThreeAxisLinearAccelerometerMeasure(0,
                                                                                       acceleration,
                                                                                       timestamp));

    ASSERT_EQ(acceleration.size(), 3);
    ASSERT_GT(timestamp, 0.0);

    // Verify acceleration values are finite
    for (size_t i = 0; i < acceleration.size(); i++)
    {
        ASSERT_TRUE(std::isfinite(acceleration[i]))
            << "Linear acceleration component " << i << " has invalid value: " << acceleration[i];
    }
}

TEST_F(BaseStateFixture, CheckThreeAxisAngularAccelerometers)
{
    runSimulationAndWaitForData();
    ASSERT_TRUE(configured);

    // Test IThreeAxisAngularAccelerometers interface
    ASSERT_EQ(iThreeAxisAngularAccelerometers->getNrOfThreeAxisAngularAccelerometers(), 1);

    std::string sensorName;
    ASSERT_TRUE(
        iThreeAxisAngularAccelerometers->getThreeAxisAngularAccelerometerName(0, sensorName));
    ASSERT_FALSE(sensorName.empty());

    ASSERT_EQ(iThreeAxisAngularAccelerometers->getThreeAxisAngularAccelerometerStatus(0),
              yarp::dev::MAS_OK);

    yarp::sig::Vector angularAcceleration(3);
    double timestamp;
    ASSERT_TRUE(iThreeAxisAngularAccelerometers
                    ->getThreeAxisAngularAccelerometerMeasure(0, angularAcceleration, timestamp));

    ASSERT_EQ(angularAcceleration.size(), 3);
    ASSERT_GT(timestamp, 0.0);

    // Verify angular acceleration values are finite
    for (size_t i = 0; i < angularAcceleration.size(); i++)
    {
        ASSERT_TRUE(std::isfinite(angularAcceleration[i]))
            << "Angular acceleration component " << i
            << " has invalid value: " << angularAcceleration[i];
    }
}

TEST_F(BaseStateFixture, CheckPreciselyTimedInterface)
{
    runSimulationAndWaitForData();
    ASSERT_TRUE(configured);

    // Test IPreciselyTimed interface
    yarp::os::Stamp stamp = iPreciselyTimed->getLastInputStamp();

    // Should have a valid timestamp
    EXPECT_GT(stamp.getTime(), 0.0);
}
