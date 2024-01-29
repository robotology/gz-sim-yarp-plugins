#include <gtest/gtest.h>
#include <gz/sim/TestFixture.hh>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

TEST(ClockTest, GetSimTimeAsExpected)
{
    // ARRANGE
    yarp::os::NetworkBase::setLocalMode(true);
    // Maximum verbosity helps with debugging
    gz::common::Console::SetVerbosity(4);
    // Instantiate test fixture
    gz::sim::TestFixture fixture("../../../tests/clock/model.sdf");
    const int iterations = 10;
    const int deltaTns = 1e6; // 1ms
    const int tolerance = 1e6; // 1ms
    yarp::os::BufferedPort<yarp::os::Bottle> p; // Create a port.
    p.open("/tmp_in"); // Give it a name on the network.
    yarp::os::Network::connect("/clock", "/tmp_in"); // connect two ports.

    // ACT
    fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ASSERT
    yarp::os::Bottle* b = p.read();
    auto simTime = b->get(0).asInt64();

    ASSERT_NEAR(simTime, (iterations - 1) * deltaTns, tolerance)
        << "simTime: " << simTime << std::endl;
}
