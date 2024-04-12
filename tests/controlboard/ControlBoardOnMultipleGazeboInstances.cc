#include <Common.hh>
#include <DeviceRegistry.hh>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <ostream>
#include <string>

#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/JointForceCmd.hh>

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
    std::string deviceScopedName = "model/coupled_pendulum/controlboard_plugin_device";
    auto plannedIterations = 100;
    yarp::dev::PolyDriver* driver1;
    yarp::dev::PolyDriver* driver2;
    yarp::dev::IEncoders* iEncoders1 = nullptr;
    yarp::dev::IEncoders* iEncoders2 = nullptr;
    double jointPosition1{}, jointPosition2{};
    gz::sim::TestFixture fixture1("../../../tests/controlboard/"
                                  "coupled_pendulum_multiple_gz_instances.sdf");
    gz::sim::TestFixture fixture2("../../../tests/controlboard/"
                                  "coupled_pendulum_multiple_gz_instances.sdf");
    gz::common::Console::SetVerbosity(4);

    fixture1
        .OnConfigure([&](const gz::sim::Entity& _worldEntity,
                         const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         gz::sim::EntityComponentManager& _ecm,
                         gz::sim::EventManager& /*_eventMgr*/) {
            driver1 = gzyarp::DeviceRegistry::getHandler()->getDevice(deviceScopedName);
            ASSERT_TRUE(driver1 != nullptr);
            iEncoders1 = nullptr;
            ASSERT_TRUE(driver1->view(iEncoders1));
        })
        .OnPreUpdate(
            [&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {})
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                std::cerr << "Iteration: " << _info.iterations << std::endl;
                iEncoders1->getEncoder(0, &jointPosition1);
                std::cerr << "Joint position 1: " << jointPosition1 << std::endl;
            })
        .Finalize();

    fixture2
        .OnConfigure([&](const gz::sim::Entity& _worldEntity,
                         const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         gz::sim::EntityComponentManager& _ecm,
                         gz::sim::EventManager& /*_eventMgr*/) {
            driver2 = gzyarp::DeviceRegistry::getHandler()->getDevice(deviceScopedName);
            ASSERT_TRUE(driver2 != nullptr);
            iEncoders2 = nullptr;
            ASSERT_TRUE(driver2->view(iEncoders2));
        })
        .OnPreUpdate(
            [&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {})
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                std::cerr << "Iteration: " << _info.iterations << std::endl;
                iEncoders2->getEncoder(0, &jointPosition2);
                std::cerr << "Joint position 2: " << jointPosition2 << std::endl;
            })
        .Finalize();

    ASSERT_TRUE(fixture1.Server()->Run(false, plannedIterations, false));
    ASSERT_TRUE(fixture2.Server()->Run(false, plannedIterations, false));

    while (fixture1.Server()->Running() || fixture2.Server()->Running())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ASSERT_EQ(fixture1.Server()->IterationCount(), plannedIterations);
    ASSERT_EQ(fixture2.Server()->IterationCount(), plannedIterations);

    // Check that DeviceRegistry has two devices, one for each Gazebo instance
    ASSERT_EQ(gzyarp::DeviceRegistry::getHandler()->getDevicesKeys().size(), 2);
}

} // namespace test
} // namespace gzyarp
