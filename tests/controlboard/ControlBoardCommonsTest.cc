#include "ControlBoardDriver.hh"
#include <ControlBoardData.hh>
#include <DeviceRegistry.hh>
#include <gzyarp/Common.hh>

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

// Check that multiple control board can be congfigured for the same robot model
TEST(ControlBoardCommonsTest, ConfigureMultipleControlBoards)
{
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR)
                     / "coupled_pendulum_two_controlboards.sdf";
    gz::sim::TestFixture testFixture(modelPath.string());

    bool configured = false;
    std::vector<std::string> deviceIds;
    std::vector<yarp::dev::gzyarp::ControlBoardDriver*> controlBoards;

    gz::common::Console::SetVerbosity(4);

    testFixture.OnConfigure([&](const gz::sim::Entity& _worldEntity,
                                const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                                gz::sim::EntityComponentManager& _ecm,
                                gz::sim::EventManager& /*_eventMgr*/) {
        deviceIds = DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
        // print the device ids
        for (auto& deviceId : deviceIds)
        {
            std::cerr << "Device id: " << deviceId << std::endl;
        }

        // Get the controlboard devices
        for (auto& deviceId : deviceIds)
        {
            yarp::dev::PolyDriver* deviceDriver = nullptr;
            EXPECT_TRUE(DeviceRegistry::getHandler()->getDevice(_ecm, deviceId, deviceDriver));
            yarp::dev::gzyarp::ControlBoardDriver* cbDriver = nullptr;
            auto viewOk = deviceDriver->view(cbDriver);
            if (viewOk && cbDriver)
            {
                controlBoards.push_back(cbDriver);
            }
        }
    });

    testFixture.Finalize();

    std::cerr << "Number of registered control board devices: " << controlBoards.size()
              << std::endl;
    ASSERT_EQ(controlBoards.size(), 2);
}

// Check that joint position limits are read correctly from yarp configuration
TEST(ControlBoardCommonsTest, JointPositionLimitsForMultipleJoints)
{
    auto modelPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR)
                     / "coupled_pendulum_two_joints_single_controlboard.sdf";
    gz::sim::TestFixture testFixture(modelPath.string());

    std::string deviceScopedName = "model/coupled_pendulum/controlboard_plugin_device";
    yarp::dev::PolyDriver* driver;
    yarp::dev::IControlLimits* iControlLimits = nullptr;
    IControlBoardData* iControlBoardData = nullptr;

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
        ASSERT_TRUE(driver->view(iControlLimits));
    });

    testFixture.Finalize();

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
