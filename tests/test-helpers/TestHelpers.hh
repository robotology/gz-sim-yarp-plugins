#pragma once

#include <DeviceRegistry.hh>

#include <gz/sim/EntityComponentManager.hh>

#include <iostream>
#include <string>
#include <vector>

#include <yarp/dev/PolyDriver.h>
namespace gzyarp::testing
{

class TestHelpers
{
public:
    template <typename T>
    static inline std::vector<T*> getDevicesOfType(gz::sim::EntityComponentManager& ecm)
    {
        std::vector<std::string> deviceIds = DeviceRegistry::getHandler()->getDevicesKeys(ecm);

        std::vector<T*> devices{};
        for (auto& deviceId : deviceIds)
        {
            yarp::dev::PolyDriver* polyDriver = nullptr;
            if (!DeviceRegistry::getHandler()->getDevice(ecm, deviceId, polyDriver))
            {
                std::cerr << "Error while getting device " << deviceId << std::endl;
                continue;
            }

            if (!polyDriver)
            {
                std::cerr << "Error: device " << deviceId << " is nullptr" << std::endl;
                continue;
            }

            T* driver = nullptr;
            auto viewOk = polyDriver->view(driver);
            if (viewOk && driver)
            {
                devices.push_back(driver);
            }

            if (viewOk && !driver)
            {
                std::cerr << "Error: view of device " << deviceId << " is nullptr" << std::endl;
            }
        }
        std::cerr << "Total devices registered: " << deviceIds.size()
                  << ", of which of selected type: " << devices.size() << std::endl;
        return devices;
    }
};

} // namespace gzyarp::testing
