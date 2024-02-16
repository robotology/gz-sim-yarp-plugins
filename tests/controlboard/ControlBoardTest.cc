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
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

TEST(ControlBoardTest, GetTorqueWithPendulumJointRelativeToParentLink)
{
    // ARRANGE

    yarp::os::NetworkBase::setLocalMode(true);
    gz::common::Console::SetVerbosity(4);

    gz::sim::TestFixture fixture("../../../tests/controlboard/"
                                 "pendulum_joint_relative_to_parent_link.sdf");

    double motorTorque{0.1};
    double linkMass{1};
    double linkLength{1.0};
    double linkInertiaAtLinkEnd{0.3352};
    int plannedIterations{50};
    int iterations{0};
    bool configured{false};
    gz::math::Vector3d gravity;
    gz::sim::Entity modelEntity;
    gz::sim::Model model;
    gz::sim::Link link;
    gz::sim::Joint joint;
    yarp::os::Property option;
    auto deviceScopedName = "model/single_pendulum/controlboard_plugin_device";
    double jointVelocityPreviousStep{};

    yarp::dev::PolyDriver* driver;
    yarp::dev::ITorqueControl* iTorqueControl = nullptr;

    fixture
        .
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

            driver = gzyarp::Handler::getHandler()->getDevice(deviceScopedName);
            ASSERT_TRUE(driver != nullptr);
            iTorqueControl = nullptr;
            ASSERT_TRUE(driver->view(iTorqueControl));

            // Get link
            auto linkEntity = model.LinkByName(_ecm, "upper_link");
            EXPECT_NE(gz::sim::kNullEntity, linkEntity);
            link = gz::sim::Link(linkEntity);
            // TODO get mass from SDF
            link.EnableVelocityChecks(_ecm, true);
            link.EnableAccelerationChecks(_ecm, true);

            // Get joint
            auto jointEntity = model.JointByName(_ecm, "upper_joint");
            EXPECT_NE(gz::sim::kNullEntity, jointEntity);
            joint = gz::sim::Joint(jointEntity);
            joint.EnablePositionCheck(_ecm, true);
            joint.EnableVelocityCheck(_ecm, true);
            joint.EnableTransmittedWrenchCheck(_ecm, true);

            configured = true;
            std::cerr << "========== test configured" << std::endl;
        })
        .OnPreUpdate([&](const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) {
            std::cerr << "========== Test PreUpdate" << std::endl;

            // Set joint torque
            joint.SetForce(_ecm, std::vector<double>{motorTorque});
            std::cerr << "========== Test PreUpdate done" << std::endl;
        })
        .OnPostUpdate(
            [&](const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) {
                std::cerr << "========== Test PostUpdate" << std::endl;

                // Use post-update callback to get values at the end of every
                // iteration

                // Get link angular position, velocity and acceleration
                auto theta = link.WorldPose(_ecm).value().Roll();
                auto theta_dot = link.WorldAngularVelocity(_ecm).value().X();
                auto theta_ddot = link.WorldAngularAcceleration(_ecm).value().X();

                // Get joint position and velocity
                auto joint_position = joint.Position(_ecm).value().at(0);
                auto joint_velocity = joint.Velocity(_ecm).value().at(0);
                double joint_acceleration{0};
                if (std::abs(joint_velocity) < 1e-6 && std::abs(jointVelocityPreviousStep) < 1e-6)
                {
                    joint_acceleration = 0;
                } else
                {
                    joint_acceleration = (joint_velocity - jointVelocityPreviousStep)
                                         / (_info.dt.count() / static_cast<double>(1e9));
                }

                // std::cerr << "delta t=" << _info.dt.count() << std::endl;
                // std::cerr << "joint acceleration: " << joint_acceleration << std::endl;
                // _ecm.ComponentData<gz::sim::components::JointForceCmd>(const Entity _entity)

                // Check that link and joint quantities are equal
                // ASSERT_NEAR(joint_position, theta, 1e-3);
                // ASSERT_NEAR(joint_velocity, theta_dot, 1e-3);

                auto expected_joint_torque
                    = linkMass * gravity.Z() * linkLength / 2.0 * std::sin(joint_position)
                      + linkInertiaAtLinkEnd * joint_acceleration;

                // Get joint torque from control board
                double joint_torque{};
                iTorqueControl->getTorque(0, &joint_torque);

                std::cerr << "Torque measured: " << joint_torque
                          << " - expected: " << expected_joint_torque << std::endl;
                EXPECT_NEAR(joint_torque, expected_joint_torque, 1e-2);

                iterations++;
                jointVelocityPreviousStep = std::abs(joint_velocity) < 1e-6 ? 0 : joint_velocity;
                std::cerr << "========== Test PostUpdate done" << std::endl;
            })
        .
        // The moment we finalize, the configure callback is called
        Finalize();

    // ACT

    // Setup simulation server, this will call the post-update callbacks.
    // It also calls pre-update and update callbacks if those are being used.
    fixture.Server()->Run(true, plannedIterations, false);

    // ASSERT
    EXPECT_TRUE(configured);
    // Verify that the post update function was called 1000 times
    EXPECT_EQ(plannedIterations, iterations);
}
