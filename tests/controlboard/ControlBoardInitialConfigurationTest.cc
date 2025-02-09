#include "ControlBoardDriver.hh"
#include <ControlBoardData.hh>
#include <DeviceRegistry.hh>
#include <Common.hh>

#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <sdf/Element.hh>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

namespace gzyarp
{
namespace test
{

// Define a struct to hold the test parameters for initial configuration tests
struct ControlBoardInitialConfigurationTestsParams {
    std::string modelFileName;
    std::vector<double> expectedJointInitialPosInRad;
};

// Define the test fixture with parameters
class ControlBoardInitialConfigurationTest : public ::testing::TestWithParam<ControlBoardInitialConfigurationTestsParams> {
};

// Define the parameterized test
TEST_P(ControlBoardInitialConfigurationTest, CheckInitialConfiguration) {
    // Get the parameters for this test instance
    const ControlBoardInitialConfigurationTestsParams& params = GetParam();

    // Construct the model path
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / params.modelFileName;
    gz::sim::TestFixture testFixture(modelPath.string());

    gz::common::Console::SetVerbosity(4);

    yarp::dev::IEncoders* iencs = nullptr;

    gz::common::Console::SetVerbosity(4);

    testFixture.OnConfigure([&](const gz::sim::Entity& _worldEntity,
                                const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                                gz::sim::EntityComponentManager& _ecm,
                                gz::sim::EventManager& /*_eventMgr*/) {
        auto deviceIds = DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
        ASSERT_TRUE(deviceIds.size() == 1);

        yarp::dev::PolyDriver* driver = nullptr;
        EXPECT_TRUE(DeviceRegistry::getHandler()->getDevice(_ecm, deviceIds[0], driver));
        ASSERT_TRUE(driver != nullptr);
        ASSERT_TRUE(driver->view(iencs));
    });

    testFixture.Finalize();

    for (int i = 0; i < params.expectedJointInitialPosInRad.size(); i++) {
        double initialPosInDeg;
        iencs->getEncoder(i, &initialPosInDeg);
        EXPECT_DOUBLE_EQ(gzyarp::convertDegreesToRadians(initialPosInDeg), params.expectedJointInitialPosInRad[i]);
    }
}

// Instantiate the test suite with different parameters
INSTANTIATE_TEST_SUITE_P(
    ControlBoardTests,
    ControlBoardInitialConfigurationTest,
    ::testing::Values(
        ControlBoardInitialConfigurationTestsParams{"pendulum_no_initial_configuration.sdf", {0.0}},
        ControlBoardInitialConfigurationTestsParams{"pendulum_with_initial_configuration.sdf", {1.0}},
        ControlBoardInitialConfigurationTestsParams{"pendulum_with_overriden_initial_configuration.sdf", {2.0}},
        ControlBoardInitialConfigurationTestsParams{"pendulum_with_doubly_overriden_initial_configuration.sdf", {-1.0}}
    )
);

} // namespace test
} // namespace gzyarp
