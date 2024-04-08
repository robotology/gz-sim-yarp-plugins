#include <ControlBoardDataSingleton.hh>
#include <DeviceRegistry.hh>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <gz/sim/Joint.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/World.hh>

#include <vector>
#include <yarp/dev/IControlLimits.h>
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

    auto singleton = ControlBoardDataSingleton::getControlBoardHandler();
    auto keys = singleton->getControlBoardKeys();

    std::cerr << "ControlBoard singleton keys vector size: " << keys.size() << std::endl;
    ASSERT_EQ(keys.size(), 2);

    std::cerr << "ControlBoard ids: " << keys[0] << ", " << keys[1] << std::endl;
    ASSERT_TRUE(std::find(keys.begin(), keys.end(), cb1Key) != keys.end());
    ASSERT_TRUE(std::find(keys.begin(), keys.end(), cb2Key) != keys.end());
}

// Check that joint position limits are read correctly from yarp configuration
TEST(ControlBoardCommonsTest, JointPositionLimitsForMultipleJoints)
{
    std::string modelSdfName = "coupled_pendulum_two_joints_single_controlboard.sdf";
    std::string sdfPath = std::string("../../../tests/controlboard/") + modelSdfName;
    std::string deviceScopedName = "model/coupled_pendulum/controlboard_plugin_device";
    yarp::dev::PolyDriver* driver;
    yarp::dev::IControlLimits* iControlLimits;

    gz::sim::TestFixture testFixture{sdfPath};
    gz::common::Console::SetVerbosity(4);

    testFixture.Finalize();

    driver = gzyarp::DeviceRegistry::getHandler()->getDevice(deviceScopedName);
    ASSERT_TRUE(driver != nullptr);
    ASSERT_TRUE(driver->view(iControlLimits));

    auto singleton = ControlBoardDataSingleton::getControlBoardHandler();
    auto keys = singleton->getControlBoardKeys();

    std::cerr << "ControlBoard singleton keys vector size: " << keys.size() << std::endl;
    ASSERT_EQ(keys.size(), 1);

    std::cerr << "ControlBoard ids: " << keys[0] << std::endl;
    ASSERT_TRUE(std::find(keys.begin(), keys.end(), deviceScopedName) != keys.end());

    auto controlBoardData = singleton->getControlBoardData(deviceScopedName);

    auto expectedJointMaxLimits = std::vector<double>{200.0, 10.0};
    auto expectedJointMinLimits = std::vector<double>{-200.0, -10.0};

    for (int i = 0; i < expectedJointMaxLimits.size(); i++)
    {
        double maxLimit, minLimit;
        iControlLimits->getLimits(i, &minLimit, &maxLimit);
        EXPECT_DOUBLE_EQ(maxLimit, expectedJointMaxLimits[i]);
        EXPECT_DOUBLE_EQ(minLimit, expectedJointMinLimits[i]);
    }
}

} // namespace test
} // namespace gzyarp
