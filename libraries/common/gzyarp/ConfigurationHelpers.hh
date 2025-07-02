#pragma once

#include <memory>
#include <string>

#include <sdf/Element.hh>

#include <yarp/os/Property.h>
#include <yarp/robotinterface/XMLReader.h>

namespace gzyarp
{

class ConfigurationHelpers
{
public:
    static bool loadPluginConfiguration(const std::shared_ptr<const sdf::Element>& sdf,
                                        yarp::os::Property& config);

    static bool loadRobotInterfaceConfiguration(const std::shared_ptr<const sdf::Element>& sdf,
                                                yarp::robotinterface::XMLReaderResult& result);

    static bool findFile(const std::string& filename, std::string& filepath);

private:
    static bool isURI(const std::string& filepath);

    static bool
    loadYarpConfigurationFile(const std::string& yarpConfigurationFile, yarp::os::Property& config);

    static bool loadYarpConfigurationString(const std::shared_ptr<const sdf::Element>& sdf,
                                            yarp::os::Property& config);
};

} // namespace gzyarp
