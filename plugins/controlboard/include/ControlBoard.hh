#pragma once

#include "ControlBoardDataSingleton.hh"

#include <gz/sim/System.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>

namespace gzyarp
{

class ControlBoard : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate,
                     public gz::sim::ISystemReset
{

public:
    ControlBoard();
    ~ControlBoard();

    // ISystemConfigure interface
    void Configure(const Entity& _entity,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   EntityComponentManager& _ecm,
                   EventManager& _eventMgr) override;

    // ISystemPreUpdate interface
    void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;

    // ISystemPostUpdate interface
    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override;

    // ISystemReset interface
    void Reset(const UpdateInfo& _info, EntityComponentManager& _ecm) override;

private:
    bool m_deviceRegistered;
    std::string m_modelScopedName;
    std::string m_deviceScopedName;
    gz::sim::Entity m_modelEntity;
    yarp::dev::PolyDriver m_controlBoardDriver;
    ControlBoardData m_ControlBoardData;
    yarp::os::Network m_yarpNetwork;
};

} // namespace gzyarp
