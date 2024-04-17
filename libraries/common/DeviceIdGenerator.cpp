#include <DeviceIdGenerator.hh>

#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

namespace gzyarp
{
using gz::sim::Entity;
using gz::sim::EntityComponentManager;

std::string DeviceIdGenerator::generateDeviceId(const Entity& entity,
                                                const EntityComponentManager& ecm,
                                                const std::string& yarpDeviceName)
{
    auto scopedName = gz::sim::scopedName(entity, ecm, "/");
    auto ecmPtr = &ecm;
    std::stringstream ss;
    ss << ecmPtr;
    return ss.str() + "/" + scopedName + "/" + yarpDeviceName;
}

} // namespace gzyarp
