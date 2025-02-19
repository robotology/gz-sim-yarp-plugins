#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/TestFixture.hh>

#include <yarp/conf/environment.h>
#include <yarp/dev/Drivers.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/Network.h>

// This global flags are set by devices to easily check which devices were loaded
std::atomic<bool> g_device1WasLoaded = false;
std::atomic<bool> g_device2WasLoaded = false;

void resetLoadedFlags()
{
    g_device1WasLoaded = false;
    g_device2WasLoaded = false;
}

class gsypRobotInterfaceTestDevice1 : public yarp::dev::DeviceDriver
{
public:
    gsypRobotInterfaceTestDevice1() = default;
    ~gsypRobotInterfaceTestDevice1() override = default;

    bool open(yarp::os::Searchable&) override
    {
        g_device1WasLoaded = true;
        return true;
    }

    bool close() override
    {
        return true;
    }
};

class gsypRobotInterfaceTestDevice2 : public yarp::dev::DeviceDriver
{
public:
    gsypRobotInterfaceTestDevice2() = default;
    ~gsypRobotInterfaceTestDevice2() override = default;

    bool open(yarp::os::Searchable&) override
    {
        g_device2WasLoaded = true;
        return true;
    }

    bool close() override
    {
        return true;
    }
};



TEST(RobotInterfaceTest, CheckEnableAndDisableTags)
{
    yarp::os::NetworkBase::setLocalMode(true);

    // Add devices used for tests
    ::yarp::dev::Drivers::factory().add(
        new ::yarp::dev::DriverCreatorOf<gsypRobotInterfaceTestDevice1>("gsypRobotInterfaceTestDevice1",
                                                                         "",
                                                                         ""));
    ::yarp::dev::Drivers::factory().add(
        new ::yarp::dev::DriverCreatorOf<gsypRobotInterfaceTestDevice2>("gsypRobotInterfaceTestDevice2",
                                                                         "",
                                                                         ""));

    resetLoadedFlags();
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "robotinterface_with_default_tags.sdf";
    std::cerr << "Testing world " << modelPath << std::endl;
    gz::sim::TestFixture fixtureDefault(modelPath.string());
    fixtureDefault.Finalize();
    EXPECT_FALSE(g_device1WasLoaded);
    EXPECT_FALSE(g_device2WasLoaded);

    resetLoadedFlags();
    modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "robotinterface_with_enable_device1.sdf";
    std::cerr << "Testing world " << modelPath << std::endl;
    gz::sim::TestFixture fixtureEnableDevice1(modelPath.string());
    fixtureEnableDevice1.Finalize();
    EXPECT_TRUE(g_device1WasLoaded);
    EXPECT_FALSE(g_device2WasLoaded);

    resetLoadedFlags();
    modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "robotinterface_with_enable_device1_and_device2.sdf";
    std::cerr << "Testing world " << modelPath << std::endl;
    gz::sim::TestFixture fixtureEnableDevice1andDevice2(modelPath.string());
    fixtureEnableDevice1andDevice2.Finalize();
    EXPECT_TRUE(g_device1WasLoaded);
    EXPECT_TRUE(g_device2WasLoaded);

    resetLoadedFlags();
    modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "robotinterface_with_enable_device1_and_device2_disable_device1.sdf";
    std::cerr << "Testing world " << modelPath << std::endl;
    gz::sim::TestFixture fixtureEnableDevice1andDevice2DisableDevice1(modelPath.string());
    fixtureEnableDevice1andDevice2DisableDevice1.Finalize();
    EXPECT_FALSE(g_device1WasLoaded);
    EXPECT_TRUE(g_device2WasLoaded);
}

