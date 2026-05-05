#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <thread>

#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/sim/TestFixture.hh>

#include <yarp/dev/ISimulatedWorld.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#define CHECK(x) EXPECT_EQ(x,true)

TEST(LaserTest, PluginTest)
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
    option.put("device", "simulatedWorld_nwc_yarp");
    option.put("remote", "/simulatedWorld_nws");
    option.put("local",  "/simulatedWorld_nwc");
    yarp::dev::PolyDriver driver;

    ASSERT_TRUE(driver.open(option));
    yarp::dev::ISimulatedWorld* isim = nullptr;
    ASSERT_TRUE(driver.view(isim));

    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    std::string frame_name;
    yarp::sig::Pose6D pose;
    yarp::sig::ColorRGB color;
    bool gravity_enabled = true;
    bool collision_enabled = true;

    ret = isim->makeSphere("sphere1", 0.1, pose, color,frame_name,gravity_enabled,collision_enabled);
    CHECK(ret);

    ret = isim->makeBox("box1",1,1,1, pose, color,frame_name,gravity_enabled,collision_enabled);
    CHECK(ret);

    ret = isim->makeCylinder("cyl1",1,1, pose, color,frame_name,gravity_enabled,collision_enabled);
    CHECK(ret);

    ret = isim->makeFrame("frame",1, pose, color,frame_name,gravity_enabled,collision_enabled);
    CHECK(ret);

    ret = isim->makeModel("model1", "filename", pose, frame_name,gravity_enabled,collision_enabled);
    CHECK(ret);

    ret = isim->deleteObject("sphere1");
    CHECK(ret);

    ret = isim->deleteAll();
    CHECK(ret);

    ret = isim->setPose("box1", pose);
    CHECK(ret);

    ret = isim->getPose("box1", pose);
    CHECK(ret);

    ret = isim->rename("box1", "box2");
    CHECK(ret);

    std::vector<std::string> objects;
    ret = isim->getList(objects);
    CHECK(ret);

    ret = isim->enableGravity("cyl1", true);
    CHECK(ret);
    ret = isim->enableGravity("cyl1", false);
    CHECK(ret);

    ret = isim->enableCollision("cyl1", true);
    CHECK(ret);
    ret = isim->enableCollision("cyl1", false);
    CHECK(ret);

    ret = isim->changeColor("box1", color);
    CHECK(ret);

    ret = isim->attach("box1", "frame1");
    CHECK(ret);

    ret = isim->detach("box1");
    CHECK(ret);
/*
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
*/
}
