#include <Common.hh>
#include <Handler.hh>

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
    auto plannedIterations = 100;
    gz::sim::TestFixture fixture1("../../../tests/controlboard/coupled_pendulum_no_plugins.sdf");
    gz::sim::TestFixture fixture2("../../../tests/controlboard/coupled_pendulum_no_plugins.sdf");
    gz::common::Console::SetVerbosity(4);

    fixture1
        .OnPreUpdate(
            [&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {})
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                std::cerr << _info.iterations << std::endl;
            })
        .Finalize();

    fixture2
        .OnPreUpdate(
            [&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {})
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                std::cerr << _info.iterations << std::endl;
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
}

} // namespace test
} // namespace gzyarp
