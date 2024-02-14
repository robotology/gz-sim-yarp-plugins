#include "../include/ControlBoard.hh"

#include "../../../libraries/singleton-devices/Handler.hh"
#include "../include/ControlBoardDataSingleton.hh"
#include "../include/ControlBoardDriver.hh"

#include <cstdlib>
#include <exception>
#include <gz/math/Vector3.hh>
#include <gz/msgs/details/wrench.pb.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <sdf/Element.hh>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{
using gz::msgs::Wrench;

ControlBoard::ControlBoard()
    : m_deviceRegistered(false)
{
}

ControlBoard::~ControlBoard()
{
    if (m_deviceRegistered)
    {
        Handler::getHandler()->removeDevice(m_deviceScopedName);
        m_deviceRegistered = false;
    }

    if (m_controlBoardDriver.isValid())
    {
        m_controlBoardDriver.close();
    }
    // ControlBoardDataSingleton::getControlBoardHandler()->removeControlBoard();
}

void ControlBoard::Configure(const Entity& _entity,
                             const std::shared_ptr<const sdf::Element>& _sdf,
                             EntityComponentManager& _ecm,
                             EventManager& _eventMgr)
{
    using ControlBoardDriverCreator
        = ::yarp::dev::DriverCreatorOf<yarp::dev::gzyarp::ControlBoardDriver>;
    yarp::dev::Drivers::factory().add(
        new ControlBoardDriverCreator("gazebo_controlboard", "", "ControlBoardDriver"));

    bool wipe = false;

    if (_sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_path = _sdf->Get<std::string>("yarpConfigurationFile");
        m_pluginParameters.fromConfigFile(ini_file_path.c_str(), wipe);
        if (!m_pluginParameters.check("yarpDeviceName"))
        {
            yError() << "gz-sim-yarp-controlboard-system : missing yarpDeviceName parameter";
            return;
        }

        yInfo() << "gz-sim-yarp-controlboard-system: configuration of device "
                << m_pluginParameters.find("yarpDeviceName").asString()
                << " loaded from yarpConfigurationFile : " << ini_file_path << "\n";
    } else
    {
        yError() << "gz-sim-yarp-controlboard-system : missing yarpConfigurationFile element";
        return;
    }

    std::string deviceName = m_pluginParameters.find("yarpDeviceName").asString();

    m_robotScopedName = gz::sim::scopedName(_entity, _ecm, "/");
    yDebug() << "gz-sim-yarp-controlboard-system : robot scoped name: " << m_robotScopedName;

    m_deviceScopedName
        = m_robotScopedName + "/" + m_pluginParameters.find("yarpDeviceName").asString();
    yDebug() << "gz-sim-yarp-controlboard-system : device scoped name: " << m_deviceScopedName;

    m_modelEntity = _entity;

    m_controlBoardData.modelScopedName = m_robotScopedName;

    m_pluginParameters.put(yarp::dev::gzyarp::YarpControlBoardScopedName.c_str(),
                           m_robotScopedName.c_str());

    // Insert the pointer in the singleton handler for retrieving it in the yarp driver
    ControlBoardDataSingleton::getControlBoardHandler()->setControlBoardData(&(m_controlBoardData));

    m_pluginParameters.put("device", "gazebo_controlboard");
    m_pluginParameters.put("name", m_deviceScopedName);
    m_pluginParameters.put("robotScopedName", m_robotScopedName);

    if (_sdf->HasElement("initialConfiguration"))
    {
        std::string initialConfiguration = _sdf->Get<std::string>("initialConfiguration");
        yDebug() << "gz-sim-yarp-controlboard-system: initialConfiguration: "
                 << initialConfiguration;
        m_pluginParameters.put("initialConfiguration", initialConfiguration);
    }

    if (!m_controlBoardDriver.open(m_pluginParameters))
    {
        yError() << "gz-sim-yarp-controlboard-system Plugin failed: error in opening yarp "
                    "driver";
        return;
    }

    if (!Handler::getHandler()->setDevice(m_deviceScopedName, &m_controlBoardDriver))
    {
        yError() << "gz-sim-yarp-basestate-system: failed setting scopedDeviceName(="
                 << m_deviceScopedName << ")";
        return;
    }

    if (!setJointProperties(_ecm))
    {
        yError() << "gz-sim-yarp-controlboard-system: failed setting joint properties";
        return;
    }

    yInfo() << "Registered YARP device with instance name:" << m_deviceScopedName;
    m_deviceRegistered = true;
}

void ControlBoard::PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    if (!updateReferences(_ecm))
    {
        yError() << "Error while updating control references";
    }
}

void ControlBoard::PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm)
{
    updateSimTime(_info);

    if (!readJointsMeasurements(_ecm))
    {
        yError() << "Error while reading joints measurements";
    }

    // checkForJointsHwFault();
}

void ControlBoard::Reset(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    // TODO
}

// Private methods

bool ControlBoard::setJointProperties(EntityComponentManager& _ecm)
{
    yarp::os::Bottle jointsFromConfig = m_pluginParameters.findGroup("jointNames");

    if (jointsFromConfig.isNull())
    {
        yError() << "Error while reading jointNames from plugin parameters";
        return false;
    }

    auto jointsFromConfigNum = jointsFromConfig.size() - 1; // -1 to exclude the group name
    yInfo() << "Found " + std::to_string(jointsFromConfigNum)
                   + " joints from the plugin configuration.";

    {
        std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

        m_controlBoardData.joints.resize(jointsFromConfigNum);

        auto model = Model(m_modelEntity);
        auto jointEntititesCount = model.JointCount(_ecm);
        yInfo() << "Found " + std::to_string(jointEntititesCount)
                       + " joints from the model description.";

        m_controlBoardData.joints.clear();
        for (size_t i = 0; i < jointsFromConfigNum; i++)
        {
            auto jointFromConfigName = jointsFromConfig.get(i + 1).asString();

            auto jointEntity = model.JointByName(_ecm, jointFromConfigName);
            if (!jointEntity)
            {
                yError() << "Joint " << jointFromConfigName
                         << " not found in the model description.";
                return false;
            }

            // Enable position, velocity and wrench check
            auto gzJoint = Joint(jointEntity);
            gzJoint.EnablePositionCheck(_ecm, true);
            gzJoint.EnableVelocityCheck(_ecm, true);
            gzJoint.EnableTransmittedWrenchCheck(_ecm, true);

            // Initialize JointProperties object
            JointProperties jointProperties{};
            jointProperties.name = jointFromConfigName;
            jointProperties.interactionMode = yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF;
            jointProperties.controlMode = VOCAB_CM_IDLE;
            jointProperties.refTorque = 0.0;
            jointProperties.torque = 0.0;
            jointProperties.maxTorqueAbs = 0.0;
            jointProperties.zeroPosition = 0.0;
            jointProperties.position = 0.0;

            m_controlBoardData.joints.push_back(jointProperties);
            yInfo() << "Joint " << jointFromConfigName << " added to the control board data.";
        }
    }

    return true;
}

void ControlBoard::updateSimTime(const gz::sim::v7::UpdateInfo& _info)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    m_controlBoardData.simTime.update(_info.simTime.count() / 1e9);
}

bool ControlBoard::readJointsMeasurements(const gz::sim::EntityComponentManager& _ecm)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    auto model = Model(m_modelEntity);
    Joint gzJoint;
    for (auto& joint : m_controlBoardData.joints)
    {
        try
        {
            gzJoint = Joint(model.JointByName(_ecm, joint.name));
        } catch (const std::exception& e)
        {
            yError() << "Error while trying to access joint " << joint.name;
            return false;
        }

        if (gzJoint.Position(_ecm).has_value())
        {
            // TODO manage unit conversions
            joint.position = gzJoint.Position(_ecm).value().at(0);
        } else
        {
            yError() << "Error while reading position for joint " << joint.name;
            return false;
        }

        if (gzJoint.Velocity(_ecm).has_value())
        {
            joint.velocity = gzJoint.Velocity(_ecm).value().at(0);
        } else
        {
            yError() << "Error while reading velocity for joint " << joint.name;
            return false;
        }

        if (gzJoint.TransmittedWrench(_ecm).has_value())
        {
            // joint.torque = gzJoint.TransmittedWrench(_ecm).value().at(0).torque().x();
            joint.torque
                = getJointTorqueFromTransmittedWrench(gzJoint,
                                                      gzJoint.TransmittedWrench(_ecm).value().at(0),
                                                      _ecm);
        } else
        {
            yError() << "Error while reading torque for joint " << joint.name;
            return false;
        }
    }
    return true;
}

double
ControlBoard::getJointTorqueFromTransmittedWrench(const Joint& gzJoint,
                                                  const Wrench& wrench,
                                                  const gz::sim::EntityComponentManager& ecm) const
{

    auto axis = gzJoint.Axis(ecm).value().at(0).Xyz();
    // TODO manage different types of joints

    // Revolute Joint
    // Motion subspace s = [0_3; axis]
    auto wrench_torque
        = gz::math::Vector3d(wrench.torque().x(), wrench.torque().y(), wrench.torque().z());
    double torque = wrench_torque.Dot(axis);
    return torque;
}

void ControlBoard::checkForJointsHwFault()
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    for (auto& joint : m_controlBoardData.joints)
    {
        if (joint.controlMode != VOCAB_CM_HW_FAULT && std::abs(joint.torque) > joint.maxTorqueAbs)
        {
            joint.controlMode = VOCAB_CM_HW_FAULT;
            yError() << "An hardware fault occurred on joint " << joint.name
                     << " torque too big! ( " << joint.torque << " )";
        }
    }
}

bool ControlBoard::updateReferences(gz::sim::EntityComponentManager& _ecm)
{
    std::lock_guard<std::mutex> lock(m_controlBoardData.mutex);

    double forceReference{};
    Joint gzJoint;
    for (auto& joint : m_controlBoardData.joints)
    {
        switch (joint.controlMode)
        {
        case VOCAB_CM_TORQUE:
            forceReference = joint.refTorque;
            break;
        case VOCAB_CM_IDLE:
        case VOCAB_CM_HW_FAULT:
            forceReference = 0.0;
            break;
        default:
            yWarning() << "Joint " << joint.name << " control mode " << joint.controlMode
                       << " is currently not implemented";
            return false;
        }

        try
        {
            gzJoint = Joint(Model(m_modelEntity).JointByName(_ecm, joint.name));
        } catch (const std::exception& e)
        {
            yError() << "Error while trying to access joint " << joint.name;
            return false;
        }

        std::vector<double> forceVec{forceReference};

        gzJoint.SetForce(_ecm, forceVec);
    }

    return true;
}

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::ControlBoard,
              gz::sim::System,
              gzyarp::ControlBoard::ISystemConfigure,
              gzyarp::ControlBoard::ISystemPreUpdate,
              gzyarp::ControlBoard::ISystemPostUpdate,
              gzyarp::ControlBoard::ISystemReset)
