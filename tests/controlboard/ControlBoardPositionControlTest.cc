#include <Common.hh>
#include <DeviceRegistry.hh>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
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
        : testFixture{"../../../tests/controlboard/pendulum_joint_relative_to_parent_link.sdf"}
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

                auto deviceKeys = gzyarp::DeviceRegistry::getHandler()->getDevicesKeys();
                ASSERT_EQ(deviceKeys.size(), 1);
                driver = gzyarp::DeviceRegistry::getHandler()->getDevice(deviceKeys[0]);

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

} // namespace test
} // namespace gzyarp
