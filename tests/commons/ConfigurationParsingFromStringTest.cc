#include <gtest/gtest.h>

#include <TestHelpers.hh>

#include <gz/common/Console.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/TestFixture.hh>
#include <sdf/Element.hh>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/Lidar2DDeviceBase.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>

TEST(ConfigurationParsingTest, LoadPluginsWithYarpConfigurationString)
{
    auto modelPath
        = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model_configuration_string.sdf";
    gz::sim::TestFixture testFixture(modelPath.string());
    yarp::os::NetworkBase::setLocalMode(true);
    gz::common::Console::SetVerbosity(4);

    gz::sim::EntityComponentManager* ecm;

    testFixture.OnConfigure([&](const gz::sim::Entity& _worldEntity,
                                const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                                gz::sim::EntityComponentManager& _ecm,
                                gz::sim::EventManager& /*_eventMgr*/) { ecm = &_ecm; });

    testFixture.Finalize();

    // Test Camera
    std::cerr << "Testing Camera configuration" << std::endl;
    auto cameraDrivers
        = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::IFrameGrabberImage>(*ecm);
    EXPECT_EQ(cameraDrivers.size(), 1);
    EXPECT_TRUE(cameraDrivers[0] != nullptr);

    // Test ForceTorque
    std::cerr << "Testing FT configuration" << std::endl;
    auto ftDrivers
        = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::ISixAxisForceTorqueSensors>(
            *ecm);
    EXPECT_EQ(ftDrivers.size(), 1);
    EXPECT_TRUE(ftDrivers[0] != nullptr);

    // Test Imu
    std::cerr << "Testing Imu configuration" << std::endl;
    auto imuDrivers
        = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::IThreeAxisGyroscopes>(*ecm);
    EXPECT_EQ(imuDrivers.size(), 1);
    EXPECT_TRUE(imuDrivers[0] != nullptr);

    // Test Laser
    std::cerr << "Testing Laser configuration" << std::endl;
    auto laserDrivers
        = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::Lidar2DDeviceBase>(*ecm);
    EXPECT_EQ(laserDrivers.size(), 1);
    EXPECT_TRUE(laserDrivers[0] != nullptr);

    // Test basestate
    std::cerr << "Testing BaseState configuration" << std::endl;
    auto bsDrivers
        = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::IPreciselyTimed>(*ecm);
    EXPECT_EQ(bsDrivers.size(), 1);
    EXPECT_TRUE(bsDrivers[0] != nullptr);

    // Controlboard test skipped since configuration too complex to pass as string
}
