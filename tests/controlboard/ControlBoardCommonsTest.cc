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
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Joint.hh>

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
    std::vector<yarp::dev::IEncoders*> controlBoards;

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
            yarp::dev::IEncoders* cbDriver = nullptr;
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

    ASSERT_TRUE(testFixture.Server()->RunOnce(/*paused=*/true));
    ASSERT_TRUE(testFixture.Server()->Run(/*blocking=*/true, 100, /*paused=*/false));
}

// Harmonic does not load the default Physics system when the world explicitly
// loads another system. Joint measurement components then exist but stay empty.
TEST(ControlBoardCommonsTest, MissingPhysicsDoesNotCrash)
{
    gz::sim::ServerConfig config;
    ASSERT_TRUE(config.SetSdfString(R"(
        <sdf version="1.7">
            <world name="missing_physics">
                <gz:policies>
                    <include_server_config_plugins>false</include_server_config_plugins>
                </gz:policies>
                <plugin filename="gz-sim-scene-broadcaster-system"
                        name="gz::sim::systems::SceneBroadcaster"/>
                <model name="pendulum1">
                    <include><uri>model://single_pendulum_no_initial_configuration.sdf</uri></include>
                </model>
                <model name="pendulum2">
                    <include><uri>model://single_pendulum_no_initial_configuration.sdf</uri></include>
                </model>
            </world>
        </sdf>)"));
    gz::sim::TestFixture testFixture(config);
    std::vector<yarp::dev::IEncoders*> encoders;
    std::vector<gz::sim::Joint> joints;
    unsigned int updates = 0;

    testFixture.OnConfigure([&](const gz::sim::Entity&,
                                const std::shared_ptr<const sdf::Element>&,
                                gz::sim::EntityComponentManager& ecm,
                                gz::sim::EventManager&) {
        const auto deviceIds = DeviceRegistry::getHandler()->getDevicesKeys(ecm);
        ASSERT_EQ(deviceIds.size(), 2);
        for (const auto& deviceId : deviceIds)
        {
            yarp::dev::PolyDriver* driver = nullptr;
            ASSERT_TRUE(DeviceRegistry::getHandler()->getDevice(ecm, deviceId, driver));
            ASSERT_NE(driver, nullptr);
            yarp::dev::IEncoders* encoder = nullptr;
            ASSERT_TRUE(driver->view(encoder));
            encoders.push_back(encoder);
        }
        ecm.Each<gz::sim::components::Joint>(
            [&](const gz::sim::Entity& entity, const gz::sim::components::Joint*) {
                gz::sim::Joint joint(entity);
                if (joint.Name(ecm).value() == "upper_joint")
                {
                    joints.push_back(joint);
                }
                return true;
            });
    });
    testFixture.OnPostUpdate([&](const gz::sim::UpdateInfo&,
                                 const gz::sim::EntityComponentManager& ecm) {
        for (const auto& joint : joints)
        {
            const auto position = joint.Position(ecm);
            const auto velocity = joint.Velocity(ecm);
            ASSERT_TRUE(position.has_value());
            ASSERT_TRUE(position->empty());
            ASSERT_TRUE(velocity.has_value());
            ASSERT_TRUE(velocity->empty());
        }
        ++updates;
    });
    testFixture.Finalize();
    ASSERT_EQ(encoders.size(), 2);
    ASSERT_EQ(joints.size(), 2);

    ASSERT_TRUE(testFixture.Server()->Run(/*blocking=*/true, 10, /*paused=*/false));
    EXPECT_EQ(updates, 10);
    for (auto* encoder : encoders)
    {
        double position = 0.0;
        ASSERT_TRUE(encoder->getEncoder(0, &position));
        EXPECT_DOUBLE_EQ(position, 0.0);
    }
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
#ifdef YARP_DEV_RETURN_VALUE_IS_GE_40
        iControlLimits->getPosLimits(i, &minLimit, &maxLimit);
#else
        iControlLimits->getLimits(i, &minLimit, &maxLimit);
#endif
        EXPECT_DOUBLE_EQ(maxLimit, expectedJointMaxLimits[i]);
        EXPECT_DOUBLE_EQ(minLimit, expectedJointMinLimits[i]);
    }
}

} // namespace test
} // namespace gzyarp
