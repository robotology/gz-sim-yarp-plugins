#include <ControlBoardDataSingleton.hh>
#include <Handler.hh>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <gz/sim/Joint.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/World.hh>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>

namespace gzyarp
{
namespace test
{

// Checks that the control board can be configured without initial conditions
TEST(ControlBoardCommonsTest, ConfigureControlBoardWithoutInitialCondition)
{
    std::string modelSdfName = "pendulum_no_initial_configuration.sdf";

    gz::sim::TestFixture testFixture{"../../../tests/controlboard/" + modelSdfName};
    gz::common::Console::SetVerbosity(4);

    testFixture.Finalize();
}

// Checks that the control board can be configured without initial conditions
TEST(ControlBoardCommonsTest, ConfigureControlBoardWithInitialCondition)
{
    std::string modelSdfName = "pendulum_with_initial_configuration.sdf";

    gz::sim::TestFixture testFixture{"../../../tests/controlboard/" + modelSdfName};
    gz::common::Console::SetVerbosity(4);

    testFixture.Finalize();
}

// Check that multiple control board can be congfigured for the same robot model
TEST(ControlBoardCommonsTest, ConfigureMultipleControlBoards)
{
    std::string modelSdfName = "coupled_pendulum_two_controlboards.sdf";
    std::string sdfPath = std::string("../../../tests/controlboard/") + modelSdfName;
    std::string deviceScopedName = "model/coupled_pendulum/controlboard_plugin_device";
    std::string cb1Key = "model/coupled_pendulum/controlboard_plugin_device";
    std::string cb2Key = "model/coupled_pendulum/controlboard_plugin_device2";

    bool configured = false;
    gz::sim::Entity modelEntity;
    gz::sim::Model model;
    yarp::dev::PolyDriver* driver;

    gz::sim::TestFixture testFixture{sdfPath};

    gz::common::Console::SetVerbosity(4);

    testFixture.Finalize();

    // testFixture.Server()->Run(true, 1, false);

    auto singleton = ControlBoardDataSingleton::getControlBoardHandler();
    auto keys = singleton->getControlBoardKeys();

    std::cerr << "ControlBoard singleton keys vector size: " << keys.size() << std::endl;

    ASSERT_TRUE(configured);
    ASSERT_EQ(keys.size(), 2);
    ASSERT_TRUE(std::find(keys.begin(), keys.end(), cb1Key) != keys.end());
    ASSERT_TRUE(std::find(keys.begin(), keys.end(), cb2Key) != keys.end());
}

} // namespace test
} // namespace gzyarp
