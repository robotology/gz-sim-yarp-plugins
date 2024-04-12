#pragma once

#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace gzyarp
{

class DeviceIdGenerator
{
public:
    static std::string generateDeviceId(const gz::sim::Entity& entity,
                                        const gz::sim::EntityComponentManager& ecm,
                                        const std::string& yarpDeviceName);
};

} // namespace gzyarp
