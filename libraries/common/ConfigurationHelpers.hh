#pragma once

#include <memory>
#include <string>

#include <sdf/Element.hh>

#include <yarp/os/Property.h>

namespace gzyarp
{

class ConfigurationHelpers
{
public:
    static bool loadPluginConfiguration(const std::shared_ptr<const sdf::Element>& sdf,
                                        yarp::os::Property& config);

private:
    static bool
    loadYarpConfigurationFile(const std::string& yarpConfigurationFile, yarp::os::Property& config);

    static bool loadYarpConfigurationString(const std::shared_ptr<const sdf::Element>& sdf,
                                            yarp::os::Property& config);
};

} // namespace gzyarp
