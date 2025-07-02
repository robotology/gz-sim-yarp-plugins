#include <ControlBoardDriver.hh>
#include <DeviceRegistry.hh>
#include <gzyarp/Common.hh>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
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

class ControlBoardPositionFixture : public testing::Test
{
protected:
    // void SetUp() override
    ControlBoardPositionFixture()
        : testFixture{(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR)
                       / "pendulum_joint_relative_to_parent_link.sdf")
                          .string()}
    {
        gz::common::Console::SetVerbosity(4);

        testFixture.
            // Use configure callback to get values at startup
            OnConfigure([&](const gz::sim::Entity& _worldEntity,
                            const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                            gz::sim::EntityComponentManager& _ecm,
                            gz::sim::EventManager& /*_eventMgr*/) {
                std::cerr << "Configuring test" << std::endl;

                gz::sim::World world(_worldEntity);

                // Get model
                auto modelEntity = world.ModelByName(_ecm, "single_pendulum");
                modelEntity = world.ModelByName(_ecm, "single_pendulum");
                EXPECT_NE(gz::sim::kNullEntity, modelEntity);
                model = gz::sim::Model(modelEntity);

                auto deviceKeys = gzyarp::DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
                ASSERT_EQ(deviceKeys.size(), 1);
                EXPECT_TRUE(DeviceRegistry::getHandler()->getDevice(_ecm, deviceKeys[0], driver));
                ASSERT_TRUE(driver != nullptr);
                iPositionControl = nullptr;
                ASSERT_TRUE(driver->view(iPositionControl));
                iControlMode = nullptr;
                ASSERT_TRUE(driver->view(iControlMode));
                iEncoders = nullptr;
                ASSERT_TRUE(driver->view(iEncoders));

                // Get joint
                auto jointEntity = model.JointByName(_ecm, "upper_joint");
                EXPECT_NE(gz::sim::kNullEntity, jointEntity);
                joint = gz::sim::Joint(jointEntity);

                // Set joint in position control mode
                ASSERT_TRUE(iControlMode->setControlMode(0, VOCAB_CM_POSITION));

                // Print number of joint configured
                int nJointsConfigured{};
                ASSERT_TRUE(iPositionControl->getAxes(&nJointsConfigured));
                std::cerr << "Number of joints configured: " << nJointsConfigured << std::endl;

                configured = true;
                std::cerr << "Test configured" << std::endl;
            });
    }

    // Get SDF model name from test parameter
    gz::sim::TestFixture testFixture;
    std::string deviceScopedName = "model/single_pendulum/controlboard_plugin_device";
    double linkMass{1};
    double linkLength{1.0};
    double linkInertiaAtLinkEnd{0.3352}; // Computed with parallel axis theorem
    int plannedIterations{1000};
    int iterations{0};
    std::vector<double> trackingErrors{};
    double acceptedTolerance{5e-2};
    bool configured{false};
    gz::math::Vector3d gravity;
    gz::sim::Entity modelEntity;
    gz::sim::Model model;
    gz::sim::Joint joint;
    yarp::dev::PolyDriver* driver;
    yarp::dev::IPositionControl* iPositionControl = nullptr;
    yarp::dev::IControlMode* iControlMode = nullptr;
    yarp::dev::IEncoders* iEncoders = nullptr;
};

class ControlBoardPositionCoupledPendulumFixture : public ::testing::Test
{
protected:
    // void SetUp() override
    ControlBoardPositionCoupledPendulumFixture()
        : testFixture{(std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR)
                       / "coupled_pendulum_two_joints_coupled.sdf")
                          .string()}
    {
        gz::common::Console::SetVerbosity(4);

        testFixture.
            // Use configure callback to get values at startup
            OnConfigure([&](const gz::sim::Entity& _worldEntity,
                            const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                            gz::sim::EntityComponentManager& _ecm,
                            gz::sim::EventManager& /*_eventMgr*/) {
                std::cerr << "========== Configuring test" << std::endl;

                gz::sim::World world(_worldEntity);

                // Get model
                auto modelEntity = world.ModelByName(_ecm, "coupled_pendulum");
                modelEntity = world.ModelByName(_ecm, "coupled_pendulum");
                EXPECT_NE(gz::sim::kNullEntity, modelEntity);
                model = gz::sim::Model(modelEntity);

                auto devicesKeys = DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
                std::cerr << "Number of Devices: " << devicesKeys.size() << std::endl;
                auto cbKey = devicesKeys.at(0);
                EXPECT_TRUE(DeviceRegistry::getHandler()->getDevice(_ecm, devicesKeys[0], driver));
                std::cerr << "Driver key: " << cbKey << std::endl;
                ASSERT_TRUE(driver != nullptr);
                iPositionControl = nullptr;
                ASSERT_TRUE(driver->view(iPositionControl));
                iControlMode = nullptr;
                ASSERT_TRUE(driver->view(iControlMode));
                iEncoders = nullptr;
                ASSERT_TRUE(driver->view(iEncoders));

                // Get joint1
                auto jointEntity0 = model.JointByName(_ecm, "upper_joint");
                EXPECT_NE(gz::sim::kNullEntity, jointEntity0);
                joint0 = gz::sim::Joint(jointEntity0);

                // Get joint2
                auto jointEntity1 = model.JointByName(_ecm, "lower_joint");
                EXPECT_NE(gz::sim::kNullEntity, jointEntity1);
                joint1 = gz::sim::Joint(jointEntity1);

                // Set joint in torque control mode
                ASSERT_TRUE(iControlMode->setControlMode(0, VOCAB_CM_POSITION));
                ASSERT_TRUE(iControlMode->setControlMode(1, VOCAB_CM_POSITION));

                // Print number of joint configured
                int nJointsConfigured{};
                ASSERT_TRUE(iPositionControl->getAxes(&nJointsConfigured));
                std::cerr << "Number of joints configured: " << nJointsConfigured << std::endl;

                configured = true;
                std::cerr << "========== Test configured" << std::endl;
            });
    }

    // Get SDF model name from test parameter
    gz::sim::TestFixture testFixture;
    double linkMass{1};
    double linkLength{1.0};
    double linkInertiaAtLinkEnd{0.3352}; // Computed with parallel axis theorem
    int plannedIterations{5000};
    int iterations{0};
    std::vector<std::vector<double>> trackingErrors{2};
    double acceptedTolerance{5e-2};
    bool configured{false};
    gz::math::Vector3d gravity;
    gz::sim::Entity modelEntity;
    gz::sim::Model model;
    gz::sim::Joint joint0;
    gz::sim::Joint joint1;
    yarp::dev::PolyDriver* driver;
    yarp::dev::IPositionControl* iPositionControl = nullptr;
    yarp::dev::IControlMode* iControlMode = nullptr;
    yarp::dev::IEncoders* iEncoders = nullptr;
};


TEST_F(ControlBoardPositionFixture, CheckPositionTrackingWithTrajectoryGenerationUsingPendulumModel)
{
    auto refPosition{90.0};
    bool motionDone{false};
    double jointPosition, jointPosError;

    testFixture
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                // std::cerr << "========== Iteration: " << iterations << std::endl;

                iEncoders->getEncoder(0, &jointPosition);
                iPositionControl->checkMotionDone(0, &motionDone);

                // std::cerr << "ref position: " << refTrajectory[iterations] << std::endl;
                // std::cerr << "joint position: " << jointPosition << std::endl;

                iterations++;
            })
        .
        // The moment we finalize, the configure callback is called
        Finalize();

    int modeSet{};
    iControlMode->getControlMode(0, &modeSet);
    ASSERT_TRUE(modeSet == VOCAB_CM_POSITION);

    // Set reference position
    iPositionControl->positionMove(0, refPosition);

    // Setup simulation server, this will call the post-update callbacks.
    // It also calls pre-update and update callbacks if those are being used.
    while (!motionDone)
    {
        std::cerr << "Running server" << std::endl;
        testFixture.Server()->Run(true, plannedIterations, false);
        jointPosError = abs(refPosition - jointPosition);
        std::cerr << "Joint position error: " << jointPosError << std::endl;
    }

    // Final assertions
    ASSERT_TRUE(configured);
    ASSERT_TRUE(motionDone);

    // Verify that the final error is within the accepted tolerance
    std::cerr << "Final tracking error: " << jointPosError << std::endl;
    ASSERT_LT(jointPosError, acceptedTolerance);
}

#if defined GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN

TEST_F(ControlBoardPositionCoupledPendulumFixture, CheckPositionTrackingWithTrajectoryGenerationUsingPendulumModel)
{
    auto refPosition{90.0};
    bool motionDone0{false};
    bool motionDone1{false};
    bool motionDone2{false};
    yarp::sig::Vector jointPosition{0.0, 0.0, 0.0};
    yarp::sig::Vector jointPosError{0.0, 0.0, 0.0};

    testFixture
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                // std::cerr << "========== Iteration: " << iterations << std::endl;

                iEncoders->getEncoders(jointPosition.data());
                iPositionControl->checkMotionDone(0, &motionDone0);
                iPositionControl->checkMotionDone(1, &motionDone1);
                iPositionControl->checkMotionDone(2, &motionDone2);

                // std::cerr << "ref position: " << refTrajectory[iterations] << std::endl;
                // std::cerr << "joint position: " << jointPosition << std::endl;

                iterations++;
            })
        .
        // The moment we finalize, the configure callback is called
        Finalize();

    int modeSet0{};
    int modeSet1{};
    int modeSet2{};
    iControlMode->getControlMode(0, &modeSet0);
    iControlMode->getControlMode(1, &modeSet1);
    iControlMode->getControlMode(2, &modeSet2);
    ASSERT_TRUE(modeSet0 == VOCAB_CM_POSITION);
    ASSERT_TRUE(modeSet1 == VOCAB_CM_POSITION);
    ASSERT_TRUE(modeSet2 == VOCAB_CM_POSITION);

    // Set reference position
    iPositionControl->positionMove(1, refPosition);

    // Setup simulation server, this will call the post-update callbacks.
    // It also calls pre-update and update callbacks if those are being used.
    while (!motionDone1)
    {
        std::cerr << "Running server" << std::endl;
        testFixture.Server()->Run(true, plannedIterations, false);
        jointPosError[1] = abs(refPosition - jointPosition[1]);
        std::cerr << "Joint 0 position error: " << jointPosError[1] << std::endl;
    }

    std::cerr << "Final tracking error for joint 0: " << jointPosError[0] << std::endl;
    ASSERT_LT(jointPosError[0], acceptedTolerance);

    iPositionControl->positionMove(2, refPosition);
    while (!motionDone2)
    {
        std::cerr << "Running server" << std::endl;
        testFixture.Server()->Run(true, plannedIterations, false);
        jointPosError[1] = abs(refPosition - jointPosition[2]);
        std::cerr << "Joint 1 position error: " << jointPosError[2] << std::endl;
    }

    // Final assertions
    ASSERT_TRUE(configured);
    ASSERT_TRUE(motionDone0);
    ASSERT_TRUE(motionDone1);
    ASSERT_TRUE(motionDone2);

    // Verify that the final error is within the accepted tolerance
    std::cerr << "Final tracking error for joint 1: " << jointPosError[1] << std::endl;
    ASSERT_LT(jointPosError[1], acceptedTolerance);
    std::cerr << "Final tracking error for joint 2: " << jointPosError[2] << std::endl;
    ASSERT_LT(jointPosError[2], acceptedTolerance);
}
#endif // GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN

} // namespace test
} // namespace gzyarp
