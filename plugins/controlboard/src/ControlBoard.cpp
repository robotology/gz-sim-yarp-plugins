#include "../include/ControlBoard.hh"

#include "../../../libraries/singleton-devices/Handler.hh"
#include "../include/ControlBoardDriver.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

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

// configure
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
    // DEBUG
    m_robotScopedName = "single_pendulum";
    m_deviceScopedName
        = m_robotScopedName + "/" + m_pluginParameters.find("yarpDeviceName").asString();
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
        std::string configuration_s = _sdf->Get<std::string>("initialConfiguration");
        m_pluginParameters.put("initialConfiguration", configuration_s);
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

    m_deviceRegistered = true;
    yInfo() << "Registered YARP device with instance name:" << m_deviceScopedName;
}

// pre-update
void ControlBoard::PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    // TODO
}

// post-update
void ControlBoard::PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm)
{
    // TODO
}

// reset
void ControlBoard::Reset(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
    // TODO
}

// Private methods

bool ControlBoard::setJointProperties(const EntityComponentManager& _ecm)
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
            std::cerr << "Searching for joint: " << jointFromConfigName << "\n";

            auto jointEntity = model.JointByName(_ecm, jointFromConfigName);
            if (!jointEntity)
            {
                yError() << "Joint " << jointFromConfigName
                         << " not found in the model description.";
                return false;
            }

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

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::ControlBoard,
              gz::sim::System,
              gzyarp::ControlBoard::ISystemConfigure,
              gzyarp::ControlBoard::ISystemPreUpdate,
              gzyarp::ControlBoard::ISystemPostUpdate,
              gzyarp::ControlBoard::ISystemReset)
