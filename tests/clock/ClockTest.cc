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
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

#include "process.hpp"

TEST(ClockTest, GetSimulationTimeFromClockPort)
{
    // ARRANGE
    yarp::os::NetworkBase::setLocalMode(true);
    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);
    // Instantiate test fixture
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model.sdf";
    gz::sim::TestFixture fixture(modelPath.string());
    fixture.Finalize();

    const int iterations = 10;
    const int deltaTns = 1e6; // 1ms
    const int tolerance = 1e6; // 1ms
    yarp::os::BufferedPort<yarp::os::Bottle> p; // Create a port.
    p.open("/tmp_in"); // Give it a name on the network.
    yarp::os::Network::connect("/clock", "/tmp_in"); // connect two ports.
    int expectedSimTimeSeconds = iterations / 1e3;
    int expectedSimTimeNanoseconds = static_cast<int>(iterations * deltaTns) % 1'000'000'000;

    // ACT
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ASSERT
    yarp::os::Bottle* b = p.read();
    auto simTimeSeconds = b->get(0).asInt32();
    auto simTimeNanoseconds = b->get(1).asInt32();

    ASSERT_EQ(simTimeSeconds, expectedSimTimeSeconds);
    ASSERT_NEAR(simTimeNanoseconds, expectedSimTimeNanoseconds, tolerance);
}

TEST(ClockTest, SimulationStartsIfYARPClockAlreadySet)
{
    // This test is a regression test for
    // https://github.com/robotology/gz-sim-yarp-plugins/issues/182, in which we check that the
    // simulation starts without deadlocks even if YARP_CLOCK is set before launching it.

    // ARRANGE

    // Set YARP_CLOCK to /clock and check if test works
    yarp::conf::environment::set_string("YARP_CLOCK", "/clock");

    // In this case we do not use setLocalMode as we need to
    // ensure that the low level of YARP behave like in the case
    // of when the user uses them, i.e. using an external yarpserver
    std::string yarpserverLocation = YARP_SERVER_LOCATION;
    std::cerr << "ClockTest: launching yarpserver from " << YARP_SERVER_LOCATION << std::endl;

    std::string command = yarpserverLocation + " --write";
    std::unique_ptr<TinyProcessLib::Process> yarpserverProcess
        = std::make_unique<TinyProcessLib::Process>(command);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);
    // Instantiate test fixture
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "model.sdf";
    gz::sim::TestFixture fixture(modelPath.string());
    fixture.Finalize();

    const int iterations = 10;
    const int deltaTns = 1e6; // 1ms

    // ACT
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

    // ASSERT
    // Check if the /clock port has been correctly created
    EXPECT_TRUE(yarp::os::NetworkBase::exists("/clock")) << "Error: /clock port does not exist";

    // Close yarpserver
    yarpserverProcess->kill();
}
