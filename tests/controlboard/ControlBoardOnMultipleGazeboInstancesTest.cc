#include <Common.hh>
#include <DeviceRegistry.hh>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <gtest/gtest.h>
#include <iomanip>
#include <ios>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <sdf/Element.hh>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

namespace gzyarp
{
namespace test
{

TEST(ControlBoardOnMultipleGazeboInstances, StartConcurrentGazeboInstances)
{
    auto plannedIterations = 10'000;
    yarp::dev::PolyDriver* driver1;
    yarp::dev::PolyDriver* driver2;
    yarp::dev::IEncoders* iEncoders1 = nullptr;
    yarp::dev::IEncoders* iEncoders2 = nullptr;
    yarp::dev::IPositionControl* iPositionControl1 = nullptr;
    yarp::dev::IPositionControl* iPositionControl2 = nullptr;
    yarp::dev::IControlMode* iControlMode1 = nullptr;
    yarp::dev::IControlMode* iControlMode2 = nullptr;
    double jointPosition1{}, jointPosition2{};
    double refPosition1 = 10;
    double refPosition2 = 5;
    bool motionDone1 = false;
    bool motionDone2 = false;
    unsigned int iterationsToCompleteMotion1 = 0;
    unsigned int iterationsToCompleteMotion2 = 0;

    gz::common::Console::SetVerbosity(4);

    gz::sim::TestFixture fixture1(
        (std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "pendulum_multiple_gz_instances.sdf")
            .string());
    fixture1
        .OnConfigure([&](const gz::sim::Entity& _worldEntity,
                         const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         gz::sim::EntityComponentManager& _ecm,
                         gz::sim::EventManager& /*_eventMgr*/) {
            auto deviceIds = gzyarp::DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
            ASSERT_TRUE(deviceIds.size() == 1);
            EXPECT_TRUE(DeviceRegistry::getHandler()->getDevice(_ecm, deviceIds[0], driver1));
            ASSERT_TRUE(driver1 != nullptr);
            iEncoders1 = nullptr;
            ASSERT_TRUE(driver1->view(iEncoders1));
            iPositionControl1 = nullptr;
            ASSERT_TRUE(driver1->view(iPositionControl1));
            iControlMode1 = nullptr;
            ASSERT_TRUE(driver1->view(iControlMode1));

            // Set joint in position control mode
            ASSERT_TRUE(iControlMode1->setControlMode(0, VOCAB_CM_POSITION));
        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                iEncoders1->getEncoder(0, &jointPosition1);
                iPositionControl1->checkMotionDone(0, &motionDone1);
                if (motionDone1 && iterationsToCompleteMotion1 == 0)
                {
                    iterationsToCompleteMotion1 = fixture1.Server()->IterationCount().value();
                }
            })
        .Finalize();

    // Workaround for https://github.com/robotology/gz-sim-yarp-plugins/issues/201
    // Remove if and once https://github.com/gazebosim/gz-physics/pull/675 is merged
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    gz::sim::TestFixture fixture2(
        (std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / "pendulum_multiple_gz_instances.sdf")
            .string());
    fixture2
        .OnConfigure([&](const gz::sim::Entity& _worldEntity,
                         const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         gz::sim::EntityComponentManager& _ecm,
                         gz::sim::EventManager& /*_eventMgr*/) {
            auto deviceIds = gzyarp::DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
            ASSERT_TRUE(deviceIds.size() == 1);
            EXPECT_TRUE(DeviceRegistry::getHandler()->getDevice(_ecm, deviceIds[0], driver2));
            ASSERT_TRUE(driver2 != nullptr);
            iEncoders2 = nullptr;
            ASSERT_TRUE(driver2->view(iEncoders2));
            iPositionControl2 = nullptr;
            ASSERT_TRUE(driver2->view(iPositionControl2));
            iControlMode2 = nullptr;
            ASSERT_TRUE(driver2->view(iControlMode2));

            // Set joint in position control mode
            ASSERT_TRUE(iControlMode2->setControlMode(0, VOCAB_CM_POSITION));
        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                iEncoders2->getEncoder(0, &jointPosition2);
                iPositionControl2->checkMotionDone(0, &motionDone2);
                if (motionDone2 && iterationsToCompleteMotion2 == 0)
                {
                    iterationsToCompleteMotion2 = fixture2.Server()->IterationCount().value();
                }
            })
        .Finalize();

    // Set reference position
    iPositionControl1->positionMove(0, refPosition1);
    iPositionControl2->positionMove(0, refPosition2);

    // First do a step blocked to workaround https://github.com/robotology/gz-sim-yarp-plugins/issues/20
    // Remove if and once https://github.com/gazebosim/gz-physics/pull/675 is merged
    ASSERT_TRUE(fixture1.Server()->Run(/*blocking=*/true, 1, /*paused=*/false));
    ASSERT_TRUE(fixture2.Server()->Run(/*blocking=*/true, 1, /*paused=*/false));

    ASSERT_TRUE(fixture1.Server()->Run(/*blocking=*/false, plannedIterations-1, /*paused=*/false));
    ASSERT_TRUE(fixture2.Server()->Run(/*blocking=*/false, plannedIterations-1, /*paused=*/false));

    while (fixture1.Server()->Running() || fixture2.Server()->Running())
    {
        std::cerr << "Waiting for Gazebo simulation to finish..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    ASSERT_EQ(fixture1.Server()->IterationCount(), plannedIterations);
    ASSERT_EQ(fixture2.Server()->IterationCount(), plannedIterations);

    if (motionDone1)
    {
        std::cerr << "Motion done simulation 1 in " << iterationsToCompleteMotion1 << " iterations"
                  << std::endl;
    }

    if (motionDone2)
    {
        std::cerr << "Motion done simulation 2 in " << iterationsToCompleteMotion2 << " iterations"
                  << std::endl;
    }

    // Print final joint positions
    std::cerr << std::fixed << std::setprecision(10)
              << "Final joint position simulation 1: " << jointPosition1 << std::endl;
    std::cerr << std::fixed << std::setprecision(10)
              << "Final joint position simulation 2: " << jointPosition2 << std::endl;
}

} // namespace test
} // namespace gzyarp
