#pragma once

#include <memory>
#include <string>

#include <sdf/Element.hh>

#include <yarp/os/Property.h>

namespace gzyarp
{

bool loadYarpConfigurationFile(const std::string& _filename, yarp::os::Property& _config);
bool loadYarpConfigurationString(const std::shared_ptr<const sdf::Element>& _sdf,
                                 yarp::os::Property& _config);
bool loadPluginConfiguration(const std::shared_ptr<const sdf::Element>& _sdf,
                             yarp::os::Property& _config);

} // namespace gzyarp
