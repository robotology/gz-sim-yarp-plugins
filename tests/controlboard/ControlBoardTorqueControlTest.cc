#include <DeviceRegistry.hh>

#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

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
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

namespace gzyarp
{
namespace test
{

class ControlBoardTorqueControlFixture : public testing::TestWithParam<std::string>
{
protected:
    // void SetUp() override
    ControlBoardTorqueControlFixture()
        : testFixture{"../../../tests/controlboard/" + GetParam()}
    {
        std::cerr << "========== Test Parameter: " << GetParam() << std::endl;
        gz::common::Console::SetVerbosity(4);

        // testFixture = gz::sim::TestFixture{"../../../tests/controlboard/" + GetParam()};

        testFixture.
            // Use configure callback to get values at startup
            OnConfigure([&](const gz::sim::Entity& _worldEntity,
                            const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                            gz::sim::EntityComponentManager& _ecm,
                            gz::sim::EventManager& /*_eventMgr*/) {
                std::cerr << "========== configuring test" << std::endl;
                // Get world and gravity
                gz::sim::World world(_worldEntity);
                gravity = world.Gravity(_ecm).value();

                // Get model
                auto modelEntity = world.ModelByName(_ecm, "single_pendulum");
                modelEntity = world.ModelByName(_ecm, "single_pendulum");
                EXPECT_NE(gz::sim::kNullEntity, modelEntity);
                model = gz::sim::Model(modelEntity);

                auto deviceKeys = gzyarp::DeviceRegistry::getHandler()->getDevicesKeys(_ecm);
                ASSERT_EQ(deviceKeys.size(), 1);
                EXPECT_TRUE(DeviceRegistry::getHandler()->getDevice(_ecm, deviceKeys[0], driver));

                ASSERT_TRUE(driver != nullptr);
                iTorqueControl = nullptr;
                ASSERT_TRUE(driver->view(iTorqueControl));
                iControlMode = nullptr;
                ASSERT_TRUE(driver->view(iControlMode));

                // Get link
                auto linkEntity = model.LinkByName(_ecm, "upper_link");
                EXPECT_NE(gz::sim::kNullEntity, linkEntity);
                link = gz::sim::Link(linkEntity);
                // TODO get mass from SDF
                link.EnableVelocityChecks(_ecm, true);
                link.EnableAccelerationChecks(_ecm, true);

                // Get parent link
                auto parentLinkEntity = model.LinkByName(_ecm, "base_link");
                EXPECT_NE(gz::sim::kNullEntity, parentLinkEntity);
                parentLink = gz::sim::Link(parentLinkEntity);
                parentLink.EnableVelocityChecks(_ecm, true);
                parentLink.EnableAccelerationChecks(_ecm, true);

                // Get joint
                auto jointEntity = model.JointByName(_ecm, "upper_joint");
                EXPECT_NE(gz::sim::kNullEntity, jointEntity);
                joint = gz::sim::Joint(jointEntity);
                joint.EnablePositionCheck(_ecm, true);
                joint.EnableVelocityCheck(_ecm, true);
                joint.EnableTransmittedWrenchCheck(_ecm, true);

                // Set joint in torque control mode
                iControlMode->setControlMode(0, VOCAB_CM_TORQUE);

                configured = true;
                std::cerr << "========== test configured" << std::endl;
            });
    }

    // Get SDF model name from test parameter
    gz::sim::TestFixture testFixture;
    double motorTorque{0.5};
    double linkMass{1};
    double linkLength{1.0};
    double linkInertiaAtLinkEnd{0.3352}; // Computed with parallel axis theorem
    int plannedIterations{1000};
    int iterations{0};
    double acceptedTolerance{5e-3};
    bool configured{false};
    gz::math::Vector3d gravity;
    gz::sim::Entity modelEntity;
    gz::sim::Model model;
    gz::sim::Link link;
    gz::sim::Link parentLink;
    gz::sim::Joint joint;
    double jointVelocityPreviousStep{};
    yarp::dev::PolyDriver* driver;
    yarp::dev::ITorqueControl* iTorqueControl = nullptr;
    yarp::dev::IControlMode* iControlMode = nullptr;
};

TEST_P(ControlBoardTorqueControlFixture, CompareJointTorqueWithExpectedValueUsingPendulumModel)
{

    testFixture
        .OnPreUpdate([&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {
            std::cerr << "========== Test PreUpdate" << std::endl;

            // Set joint torque
            // joint.SetForce(_ecm, std::vector<double>{motorTorque});
            iTorqueControl->setRefTorque(0, motorTorque);
            std::cerr << "========== Test PreUpdate done" << std::endl;
        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                std::cerr << "========== Test PostUpdate" << std::endl;
                std::cerr << "iteration: " << iterations << std::endl;
                // Use post-update callback to get values at the end of every
                // iteration

                // Get joint position and velocity
                auto jointAxis = joint.Axis(_ecm).value().at(0).Xyz();
                auto jointWrench = joint.TransmittedWrench(_ecm).value().at(0).torque();
                auto jointPosition = joint.Position(_ecm).value().at(0);
                auto jointVelocity = joint.Velocity(_ecm).value().at(0);

                double jointAcceleration{0};
                if (std::abs(jointVelocity) < 1e-6 && std::abs(jointVelocityPreviousStep) < 1e-6)
                {
                    jointAcceleration = 0;
                } else
                {
                    jointAcceleration = (jointVelocity - jointVelocityPreviousStep)
                                        / (_info.dt.count() / static_cast<double>(1e9));
                }
                std::cerr << "joint axis: " << jointAxis[0] << ", " << jointAxis[1] << ", "
                          << jointAxis[2] << ", position: " << jointPosition
                          << ", joint velocity: " << jointVelocity
                          << ", joint acc: " << jointAcceleration
                          << ", joint torque: " << jointWrench.x() << ", " << jointWrench.y()
                          << ", " << jointWrench.z() << std::endl;

                // Get link kinematic quantities
                auto parentLinkPose = parentLink.WorldPose(_ecm).value();
                // Compute link pose in body frame
                auto linkPoseBody = parentLinkPose.CoordPoseSolve(link.WorldPose(_ecm).value());

                auto theta = linkPoseBody.Roll();
                auto thetaDot = link.WorldAngularVelocity(_ecm).value().X();
                auto thetaDDot = link.WorldAngularAcceleration(_ecm).value().X();
                std::cerr << "theta: " << theta << ", theta_dot: " << thetaDot
                          << ", theta_ddot: " << thetaDDot << std::endl;

                // Check that link and joint quantities are equal
                // ASSERT_NEAR(joint_position, theta, 1e-3);
                ASSERT_NEAR(jointVelocity, thetaDot, acceptedTolerance);

                // From dynamics 2nd law it is possible to compute the expected joint torque
                auto expectedJointTorque
                    = -linkMass * gravity.Z() * linkLength / 2.0 * std::sin(jointPosition)
                      + linkInertiaAtLinkEnd * thetaDDot;

                // Get joint torque from control board
                double jointTorque{};
                iTorqueControl->getTorque(0, &jointTorque);

                std::cerr << "Torque measured: " << jointTorque
                          << " - expected: " << expectedJointTorque << std::endl;

                if (iterations > 1)
                {
                    EXPECT_NEAR(jointTorque, expectedJointTorque, acceptedTolerance);
                }

                iterations++;
                jointVelocityPreviousStep = std::abs(jointVelocity) < 1e-6 ? 0 : jointVelocity;
                std::cerr << "========== Test PostUpdate done" << std::endl;
            })
        .
        // The moment we finalize, the configure callback is called
        Finalize();

    // ACT

    // Setup simulation server, this will call the post-update callbacks.
    // It also calls pre-update and update callbacks if those are being used.
    testFixture.Server()->Run(true, plannedIterations, false);

    // ASSERT
    EXPECT_TRUE(configured);
    // Verify that the post update function was called 1000 times
    EXPECT_EQ(plannedIterations, iterations);
}

INSTANTIATE_TEST_SUITE_P(ControlBoardTorqueControl,
                         ControlBoardTorqueControlFixture,
                         testing::Values("pendulum_joint_relative_to_child_link.sdf"
                                         //  ,"pendulum_joint_relative_to_parent_link.sdf"
                                         ));

} // namespace test
} // namespace gzyarp
