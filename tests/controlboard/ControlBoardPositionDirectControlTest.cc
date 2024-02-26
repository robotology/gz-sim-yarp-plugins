#include "../../libraries/common/Common.hh"
#include "../../libraries/singleton-devices/Handler.hh"
#include <cmath>
#include <gtest/gtest.h>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <iostream>
#include <string>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

class ControlBoardPositionDirectFixture : public testing::Test
{
protected:
    // void SetUp() override
    ControlBoardPositionDirectFixture()
        : testFixture{"../../../tests/controlboard/pendulum_joint_relative_to_parent_link.sdf"}
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

                driver = gzyarp::Handler::getHandler()->getDevice(deviceScopedName);
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
    std::string deviceScopedName = "model/single_pendulum/controlboard_plugin_device";
    double linkMass{1};
    double linkLength{1.0};
    double linkInertiaAtLinkEnd{0.3352}; // Computed with parallel axis theorem
    int plannedIterations{5000};
    int iterations{0};
    double acceptedTolerance{1e-2};
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

TEST_F(ControlBoardPositionDirectFixture, CheckPositionTrackingUsingPendulumModel)
{
    // Generate ref trajectory
    std::vector<double> refTrajectory(plannedIterations);
    double amplitude = 5.0; // deg
    for (int i = 0; i < plannedIterations; i++)
    {
        double t = static_cast<double>(i) / plannedIterations;
        double value = amplitude * std::sin(2 * gzyarp::pi * t);
        refTrajectory[i] = value;
    }

    double jointPosition;

    testFixture
        .OnPreUpdate([&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {
            // Set ref position
            iPositionDirectControl->setPosition(0, refTrajectory[iterations]);
        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                // std::cerr << "========== Iteration: " << iterations << std::endl;

                iEncoders->getEncoder(0, &jointPosition);

                // std::cerr << "ref position: " << refTrajectory[iterations] << std::endl;
                // std::cerr << "joint position: " << jointPosition << std::endl;

                if (iterations > 1)
                {
                    // Tracking error
                    EXPECT_NEAR(jointPosition, refTrajectory[iterations], acceptedTolerance);
                }

                iterations++;
            })
        .
        // The moment we finalize, the configure callback is called
        Finalize();

    int modeSet{};
    iControlMode->getControlMode(0, &modeSet);
    ASSERT_TRUE(modeSet == VOCAB_CM_POSITION_DIRECT);

    // // Setup simulation server, this will call the post-update callbacks.
    // // It also calls pre-update and update callbacks if those are being used.
    testFixture.Server()->Run(true, plannedIterations, false);

    // // Final assertions
    EXPECT_TRUE(configured);
    // // Verify that the post update function was called 1000 times
    EXPECT_EQ(plannedIterations, iterations);
}
