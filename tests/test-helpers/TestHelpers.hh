#pragma once

#include <iostream>
#include <vector>

#include <DeviceRegistry.hh>

namespace gzyarp::testing
{

class TestHelpers
{
public:
    template <typename T> std::vector<T*> static getDevicesOfType()
    {
        auto deviceIds = DeviceRegistry::getHandler()->getDevicesKeys();
        std::vector<T*> devices{};
        for (auto& deviceId : deviceIds)
        {
            auto polyDriver = DeviceRegistry::getHandler()->getDevice(deviceId);
            T* driver = nullptr;
            auto viewOk = polyDriver->view(driver);
            if (viewOk && driver)
            {
                devices.push_back(driver);
            }
        }
        std::cerr << "Total devices registered: " << deviceIds.size()
                  << ", of which of selected type: " << devices.size() << std::endl;
        return devices;
    }
};

} // namespace gzyarp::testing
