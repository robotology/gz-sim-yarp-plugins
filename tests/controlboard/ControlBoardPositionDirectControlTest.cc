#include <Common.hh>
#include <ControlBoardDriver.hh>
#include <DeviceRegistry.hh>
#include <TestHelpers.hh>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Types.hh>
#include <sdf/Element.hh>

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

class ControlBoardPositionDirectFixture : public ::testing::Test
{
protected:
    // void SetUp() override
    ControlBoardPositionDirectFixture()
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
                std::cerr << "========== Configuring test" << std::endl;

                gz::sim::World world(_worldEntity);

                // Get model
                auto modelEntity = world.ModelByName(_ecm, "single_pendulum");
                modelEntity = world.ModelByName(_ecm, "single_pendulum");
                EXPECT_NE(gz::sim::kNullEntity, modelEntity);
                model = gz::sim::Model(modelEntity);

                auto devicesKeys = DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
                std::cerr << "Number of Devices: " << devicesKeys.size() << std::endl;
                auto cbKey = devicesKeys.at(0);
                EXPECT_TRUE(DeviceRegistry::getHandler()->getDevice(_ecm, devicesKeys[0], driver));
                std::cerr << "Driver key: " << cbKey << std::endl;
                ASSERT_TRUE(driver != nullptr);
                iPositionDirectControl = nullptr;
                ASSERT_TRUE(driver->view(iPositionDirectControl));
                iControlMode = nullptr;
                ASSERT_TRUE(driver->view(iControlMode));
                iEncoders = nullptr;
                ASSERT_TRUE(driver->view(iEncoders));

                // Get joint
                auto jointEntity = model.JointByName(_ecm, "upper_joint");
                EXPECT_NE(gz::sim::kNullEntity, jointEntity);
                joint = gz::sim::Joint(jointEntity);

                // Set joint in torque control mode
                ASSERT_TRUE(iControlMode->setControlMode(0, VOCAB_CM_POSITION_DIRECT));

                // Print number of joint configured
                int nJointsConfigured{};
                ASSERT_TRUE(iPositionDirectControl->getAxes(&nJointsConfigured));
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
    std::vector<double> trackingErrors{};
    double acceptedTolerance{5e-2};
    bool configured{false};
    gz::math::Vector3d gravity;
    gz::sim::Entity modelEntity;
    gz::sim::Model model;
    gz::sim::Joint joint;
    yarp::dev::PolyDriver* driver;
    yarp::dev::IPositionDirect* iPositionDirectControl = nullptr;
    yarp::dev::IControlMode* iControlMode = nullptr;
    yarp::dev::IEncoders* iEncoders = nullptr;
};

class ControlBoardPositionDirectCoupledPendulumFixture : public ::testing::Test
{
protected:
    // void SetUp() override
    ControlBoardPositionDirectCoupledPendulumFixture()
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
                iPositionDirectControl = nullptr;
                ASSERT_TRUE(driver->view(iPositionDirectControl));
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
                ASSERT_TRUE(iControlMode->setControlMode(0, VOCAB_CM_POSITION_DIRECT));
                ASSERT_TRUE(iControlMode->setControlMode(1, VOCAB_CM_POSITION_DIRECT));

                // Print number of joint configured
                int nJointsConfigured{};
                ASSERT_TRUE(iPositionDirectControl->getAxes(&nJointsConfigured));
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
    yarp::dev::IPositionDirect* iPositionDirectControl = nullptr;
    yarp::dev::IControlMode* iControlMode = nullptr;
    yarp::dev::IEncoders* iEncoders = nullptr;
};

TEST_F(ControlBoardPositionDirectFixture, CheckPositionTrackingUsingPendulumModel)
{
    // Generate ref trajectory
    std::vector<double> refTrajectory(plannedIterations);
    double amplitude = 100.0; // deg
    for (int i = 0; i < plannedIterations; i++)
    {
        double t = static_cast<double>(i) / plannedIterations;
        double value = amplitude * std::sin(gzyarp::pi * t);
        refTrajectory[i] = value;
    }

    double jointPosition, jointRefPosition;

    testFixture
        .OnPreUpdate([&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {
            // Set ref position
            iPositionDirectControl->setPosition(0, refTrajectory[iterations]);
            iPositionDirectControl->getRefPosition(0, &jointRefPosition);
            EXPECT_EQ(refTrajectory[iterations], jointRefPosition);

            std::cerr << "Joint 0 ref position: " << jointRefPosition << std::endl;
            std::cerr << "Joint 0 position: " << jointPosition << std::endl;

        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                // std::cerr << "========== Iteration: " << iterations << std::endl;

                iEncoders->getEncoder(0, &jointPosition);
                // std::cerr << "ref position: " << refTrajectory[iterations] << std::endl;
                // std::cerr << "joint position: " << jointPosition << std::endl;

                // Tracking error
                // EXPECT_NEAR(jointPosition, refTrajectory[iterations], acceptedTolerance);
                trackingErrors.push_back(abs(refTrajectory[iterations] - jointPosition));

                iterations++;
            })
        .
        // The moment we finalize, the configure callback is called
        Finalize();

    int modeSet{};
    ASSERT_TRUE(iControlMode->getControlMode(0, &modeSet));
    ASSERT_TRUE(modeSet == VOCAB_CM_POSITION_DIRECT);

    // Setup simulation server, this will call the post-update callbacks.
    // It also calls pre-update and update callbacks if those are being used.
    testFixture.Server()->Run(true, plannedIterations, false);
    std::cerr << "Simulation completed" << std::endl;
    // Final assertions
    EXPECT_TRUE(configured);
    // Verify that the post update function was called 1000 times
    EXPECT_EQ(plannedIterations, iterations);
    // Verify that the average tracking error is within the accepted tolerance
    std::cerr << "Tracking error vector size: " << trackingErrors.size() << std::endl;
    auto avgTrackgingError = std::accumulate(trackingErrors.begin(), trackingErrors.end(), 0.0)
                             / trackingErrors.size();
    std::cerr << "Average tracking error: " << avgTrackgingError << std::endl;
    EXPECT_LT(avgTrackgingError, acceptedTolerance);
}

TEST_F(ControlBoardPositionDirectCoupledPendulumFixture, CheckPositionTrackingUsingCoupledPendulumModel)
{
    // Generate ref trajectory
    std::vector<double> refTrajectory(plannedIterations);
    double amplitude = 100.0; // deg
    for (int i = 0; i < plannedIterations; i++)
    {
        double t = static_cast<double>(i) / plannedIterations;
        double value = amplitude * std::sin(gzyarp::pi * t);
        refTrajectory[i] = value;
    }

    yarp::sig::Vector jointPositions{0.0, 0.0}, jointRefPositions{0.0, 0.0};

    testFixture
        .OnPreUpdate([&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {
            // Set ref position
            iPositionDirectControl->setPosition(0, refTrajectory[iterations]);
            iPositionDirectControl->setPosition(1, refTrajectory[iterations]);
            iPositionDirectControl->getRefPositions(jointRefPositions.data());
            EXPECT_EQ(refTrajectory[iterations], jointRefPositions[0]);
            EXPECT_EQ(refTrajectory[iterations], jointRefPositions[1]);
        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                // std::cerr << "========== Iteration: " << iterations << std::endl;

                iEncoders->getEncoders(jointPositions.data());

                // std::cerr << "ref position: " << refTrajectory[iterations] << std::endl;
                // std::cerr << "joint position: " << jointPosition << std::endl;

                // Tracking error
                // EXPECT_NEAR(jointPosition, refTrajectory[iterations], acceptedTolerance);
                trackingErrors[0].push_back(abs(refTrajectory[iterations] - jointPositions[0]));
                trackingErrors[1].push_back(abs(refTrajectory[iterations] - jointPositions[1]));

                iterations++;
            })
        .
        // The moment we finalize, the configure callback is called
        Finalize();

    int modeSet0{};
    int modeSet1{};
    ASSERT_TRUE(iControlMode->getControlMode(0, &modeSet0));
    ASSERT_TRUE(iControlMode->getControlMode(1, &modeSet1));
    ASSERT_TRUE(modeSet0 == VOCAB_CM_POSITION_DIRECT);
    ASSERT_TRUE(modeSet1 == VOCAB_CM_POSITION_DIRECT);

    // Setup simulation server, this will call the post-update callbacks.
    // It also calls pre-update and update callbacks if those are being used.
    testFixture.Server()->Run(true, plannedIterations, false);
    std::cerr << "Simulation completed" << std::endl;
    // Final assertions
    EXPECT_TRUE(configured);
    // Verify that the post update function was called 1000 times
    EXPECT_EQ(plannedIterations, iterations);
    // Verify that the average tracking error is within the accepted tolerance
    std::cerr << "Tracking error vector size: " << trackingErrors.size() << std::endl;
    auto avgTrackgingError0 = std::accumulate(trackingErrors[0].begin(), trackingErrors[0].end(), 0.0)
                             / trackingErrors[0].size();
    auto avgTrackgingError1 = std::accumulate(trackingErrors[1].begin(), trackingErrors[1].end(), 0.0)
                            / trackingErrors[1].size();
    std::cerr << "Average tracking error joint 0: " << avgTrackgingError0 << std::endl;
    std::cerr << "Average tracking error joint 1: " << avgTrackgingError1 << std::endl;
    EXPECT_LT(avgTrackgingError0, acceptedTolerance);
    EXPECT_LT(avgTrackgingError1, acceptedTolerance);
}

} // namespace test
} // namespace gzyarp
