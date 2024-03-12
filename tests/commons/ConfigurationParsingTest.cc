#include <gtest/gtest.h>

#include <Handler.hh>

#include <gz/sim/TestFixture.hh>

#include <iostream>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>

TEST(ConfigurationParsingTest, LoadPluginsWithYarpConfigurationString)
{
    std::string modelSdfName = "model_configuration_string.sdf";
    std::string sdfPath = std::string("../../../tests/commons/") + modelSdfName;
    yarp::os::NetworkBase::setLocalMode(true);

    gz::sim::TestFixture testFixture{sdfPath};
    gz::common::Console::SetVerbosity(4);

    testFixture.Finalize();

    // Test Camera
    std::cerr << "Testing Camera configuration" << std::endl;
    auto cameraDeviceName = "model/model_with_plugins/link/link_1/sensor/camera_sensor/"
                            "camera_plugin_device"; // sensorScopedName / yarpDeviceName
    auto driver = gzyarp::Handler::getHandler()->getDevice(cameraDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test ForceTorque
    std::cerr << "Testing FT configuration" << std::endl;
    auto ftDeviceName = "model/model_with_plugins/joint/joint_12/sensor/"
                        "force_torque_sensor/"
                        "forcetorque_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(ftDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test Imu
    std::cerr << "Testing Imu configuration" << std::endl;
    auto imuDeviceName = "model/model_with_plugins/link/link_1/sensor/imu_sensor/"
                         "imu_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(imuDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test Laser
    std::cerr << "Testing Laser configuration" << std::endl;
    auto laserDeviceName = "model/model_with_plugins/link/link_1/sensor/laser_sensor/"
                           "laser_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(laserDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test basestate
    std::cerr << "Testing BaseState configuration" << std::endl;
    auto baseStateDeviceName = "model/model_with_plugins/link/link_1/"
                               "basestate_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(baseStateDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Controlboard test skipped since configuration too complex to pass as string
}

TEST(ConfigurationParsingTest, LoadPluginsWithYarpConfigurationFile)
{
    std::string modelSdfName = "model_configuration_file.sdf";
    std::string sdfPath = std::string("../../../tests/commons/") + modelSdfName;
    yarp::os::NetworkBase::setLocalMode(true);

    gz::sim::TestFixture testFixture{sdfPath};
    gz::common::Console::SetVerbosity(4);
    yarp::dev::PolyDriver* driver = nullptr;

    testFixture.Finalize();

    // Test Camera
    std::cerr << "Testing Camera configuration" << std::endl;
    auto cameraDeviceName = "model/model_with_plugins/link/link_1/sensor/camera_sensor/"
                            "camera_plugin_device"; // sensorScopedName / yarpDeviceName
    driver = gzyarp::Handler::getHandler()->getDevice(cameraDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test ForceTorque
    std::cerr << "Testing FT configuration" << std::endl;
    auto ftDeviceName = "model/model_with_plugins/joint/joint_12/sensor/"
                        "force_torque_sensor/"
                        "forcetorque_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(ftDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test Imu
    std::cerr << "Testing Imu configuration" << std::endl;
    auto imuDeviceName = "model/model_with_plugins/link/link_1/sensor/imu_sensor/"
                         "imu_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(imuDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test Laser
    std::cerr << "Testing Laser configuration" << std::endl;
    auto laserDeviceName = "model/model_with_plugins/link/link_1/sensor/laser_sensor/"
                           "laser_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(laserDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test basestate
    std::cerr << "Testing BaseState configuration" << std::endl;
    auto baseStateDeviceName = "model/model_with_plugins/link/link_1/"
                               "basestate_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(baseStateDeviceName);
    EXPECT_TRUE(driver != nullptr);

    // Test controlboard
    std::cerr << "Testing ControlBoard configuration" << std::endl;
    auto controlboardDeviceName = "model/model_with_plugins/controlboard_plugin_device";
    driver = gzyarp::Handler::getHandler()->getDevice(controlboardDeviceName);
    EXPECT_TRUE(driver != nullptr);
}
