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

    gz::sim::TestFixture fixture1(
        (std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "dummy_sphere.sdf").string());
    gz::sim::TestFixture fixture2(
        (std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "dummy_box.sdf").string());
    gz::common::Console::SetVerbosity(4);

    fixture1.Finalize();
    fixture2.Finalize();

    ASSERT_TRUE(fixture1.Server()->Run(false, plannedIterations, false));
    ASSERT_TRUE(fixture2.Server()->Run(false, plannedIterations, false));

    while (fixture1.Server()->Running() || fixture2.Server()->Running())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cerr << "Waiting for Gazebo simulation to finish..." << std::endl;
    }

    ASSERT_EQ(fixture1.Server()->IterationCount(), plannedIterations);
    ASSERT_EQ(fixture2.Server()->IterationCount(), plannedIterations);
}
