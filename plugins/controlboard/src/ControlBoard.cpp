#include "../include/ControlBoard.hh"
#include "../../../libraries/singleton-devices/Handler.hh"

#include <gz/plugin/Register.hh>

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
    ControlBoardDataSingleton::getControlBoardHandler()->removeControlBoard();
}

// configure
void ControlBoard::Configure(const Entity& _entity,
                             const std::shared_ptr<const sdf::Element>& _sdf,
                             EntityComponentManager& _ecm,
                             EventManager& _eventMgr)
{
    // TODO

    // // Get model scoped name
    // m_modelScopedName = _ecm.Component<components::ModelScopedName>(_entity)->Data();

    // // Get device scoped name
    // m_deviceScopedName = _sdf->Get<std::string>("device_scoped_name");

    // // Get model entity
    // m_modelEntity = _ecm.EntityByComponents(components::ModelScopedName(m_modelScopedName));

    // // Get control board data
    // m_ControlBoardData =
    // ControlBoardDataSingleton::getControlBoardHandler()->getControlBoardData();

    // // Register device
    // if (!m_deviceRegistered)
    // {
    //     Handler::getHandler()->addDevice(m_deviceScopedName, m_ControlBoardData);
    //     m_deviceRegistered = true;
    // }

    // // Open control board driver
    // if (!m_controlBoardDriver.isValid())
    // {
    //     m_controlBoardDriver.open(m_deviceScopedName);
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
