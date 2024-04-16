#include <Common.hh>
#include <DeviceRegistry.hh>

#include <algorithm>
#include <chrono>
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
    double jointPosition1{}, jointPosition2{};
    gz::sim::TestFixture fixture1("../../../tests/controlboard/"
                                  "double_pendulum_multiple_gz_instances.sdf");
    gz::sim::TestFixture fixture2("../../../tests/controlboard/"
                                  "double_pendulum_multiple_gz_instances.sdf");
    gz::common::Console::SetVerbosity(4);

    fixture1
        .OnConfigure([&](const gz::sim::Entity& _worldEntity,
                         const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         gz::sim::EntityComponentManager& _ecm,
                         gz::sim::EventManager& /*_eventMgr*/) {
            auto deviceIds = gzyarp::DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
            ASSERT_TRUE(deviceIds.size() == 1);
            driver1 = gzyarp::DeviceRegistry::getHandler()->getDevice(deviceIds[0]);
            ASSERT_TRUE(driver1 != nullptr);
            iEncoders1 = nullptr;
            ASSERT_TRUE(driver1->view(iEncoders1));
        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                iEncoders1->getEncoder(0, &jointPosition1);
            })
        .Finalize();

    fixture2
        .OnConfigure([&](const gz::sim::Entity& _worldEntity,
                         const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         gz::sim::EntityComponentManager& _ecm,
                         gz::sim::EventManager& /*_eventMgr*/) {
            auto deviceIds = gzyarp::DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
            ASSERT_TRUE(deviceIds.size() == 1);
            driver2 = gzyarp::DeviceRegistry::getHandler()->getDevice(deviceIds[0]);
            ASSERT_TRUE(driver2 != nullptr);
            iEncoders2 = nullptr;
            ASSERT_TRUE(driver2->view(iEncoders2));
        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                iEncoders2->getEncoder(0, &jointPosition2);
            })
        .Finalize();

    ASSERT_TRUE(fixture1.Server()->Run(false, plannedIterations, false));
    ASSERT_TRUE(fixture2.Server()->Run(false, plannedIterations, false));

    while (fixture1.Server()->Running() || fixture2.Server()->Running())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cerr << "Waiting for Gazebo simulation to finish..." << std::endl;
    }

    ASSERT_EQ(fixture1.Server()->IterationCount(), plannedIterations);
    ASSERT_EQ(fixture2.Server()->IterationCount(), plannedIterations);

    // Check that DeviceRegistry has two devices, one for each Gazebo instance
    ASSERT_EQ(gzyarp::DeviceRegistry::getHandler()->getDevicesKeys().size(), 2);

    // Print final joint positions
    std::cerr << std::fixed << std::setprecision(6)
              << "Final joint position simulation 1: " << jointPosition1 << std::endl;
    std::cerr << std::fixed << std::setprecision(6)
              << "Final joint position simulation 2: " << jointPosition2 << std::endl;
}

} // namespace test
} // namespace gzyarp
