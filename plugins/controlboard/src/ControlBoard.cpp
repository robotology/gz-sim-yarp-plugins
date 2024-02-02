#include "../include/ControlBoard.hh"

#include "../../../libraries/singleton-devices/Handler.hh"
#include "../include/ControlBoardDriver.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

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

    yarp::os::Property driver_properties;
    bool wipe = false;

    if (_sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_path = _sdf->Get<std::string>("yarpConfigurationFile");
        driver_properties.fromConfigFile(ini_file_path.c_str(), wipe);
        if (!driver_properties.check("yarpDeviceName"))
        {
            yError() << "gz-sim-yarp-controlboard-system : missing yarpDeviceName parameter";
            return;
        }

        yInfo() << "gz-sim-yarp-controlboard-system: configuration of device "
                << driver_properties.find("yarpDeviceName").asString()
                << " loaded from yarpConfigurationFile : " << ini_file_path << "\n";
    } else
    {
        yError() << "gz-sim-yarp-controlboard-system : missing yarpConfigurationFile element";
        return;
    }

    std::string deviceName = driver_properties.find("yarpDeviceName").asString();
    m_robotScopedName = gz::sim::scopedName(_entity, _ecm, "/");
    m_deviceScopedName
        = m_robotScopedName + "/" + driver_properties.find("yarpDeviceName").asString();
    m_modelEntity = _entity;

    m_controlBoardData.modelScopedName = m_robotScopedName;

    driver_properties.put(yarp::dev::gzyarp::YarpControlBoardScopedName.c_str(),
                          m_robotScopedName.c_str());

    // Insert the pointer in the singleton handler for retrieving it in the yarp driver
    ControlBoardDataSingleton::getControlBoardHandler()->setControlBoardData(&(m_controlBoardData));

    driver_properties.put("robot", m_robotScopedName);
    driver_properties.put("device", m_deviceScopedName);

    if (!m_controlBoardDriver.open(driver_properties))
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

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::ControlBoard,
              gz::sim::System,
              gzyarp::ControlBoard::ISystemConfigure,
              gzyarp::ControlBoard::ISystemPreUpdate,
              gzyarp::ControlBoard::ISystemPostUpdate,
              gzyarp::ControlBoard::ISystemReset)
