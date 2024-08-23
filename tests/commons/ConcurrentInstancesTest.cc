#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <ostream>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/TestFixture.hh>

TEST(ConcurrentInstancesTest, StartConcurrentGazeboInstancesOfDifferentModels)
{
    auto plannedIterations = 1'000;

    gz::common::Console::SetVerbosity(4);


    gz::sim::TestFixture fixture1(
        (std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "dummy_sphere.sdf").string());
    fixture1.Finalize();

    // Workaround for https://github.com/robotology/gz-sim-yarp-plugins/issues/201

    gz::sim::TestFixture fixture2(
        (std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "dummy_box.sdf").string());
    fixture2.Finalize();

    // First do a step blocked to workaround https://github.com/robotology/gz-sim-yarp-plugins/issues/20
    // Remove if and once https://github.com/gazebosim/gz-physics/pull/675 is merged
    ASSERT_TRUE(fixture1.Server()->Run(/*blocking=*/true, 1, /*paused=*/false));
    ASSERT_TRUE(fixture2.Server()->Run(/*blocking=*/true, 1, /*paused=*/false));

    ASSERT_TRUE(fixture1.Server()->Run(/*paused=*/false, plannedIterations-1, /*paused=*/false));
    ASSERT_TRUE(fixture2.Server()->Run(/*paused=*/false, plannedIterations-1, /*paused=*/false));

    while (fixture1.Server()->Running() || fixture2.Server()->Running())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ASSERT_EQ(fixture1.Server()->IterationCount(), plannedIterations);
    ASSERT_EQ(fixture2.Server()->IterationCount(), plannedIterations);
}
