#include <ConfigurationHelpers.hh>

#include <filesystem>
#include <iostream>
#include <regex>
#include <string>

#include <gz/common/SystemPaths.hh>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gzyarp
{

bool ConfigurationHelpers::loadPluginConfiguration(const std::shared_ptr<const sdf::Element>& sdf,
                                                   yarp::os::Property& config)
{
    if (sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_path = sdf->Get<std::string>("yarpConfigurationFile");
        return loadYarpConfigurationFile(ini_file_path, config);
    }

    if (sdf->HasElement("yarpConfigurationString"))
    {
        return loadYarpConfigurationString(sdf, config);
    }

    return false;
}

// Private methods

bool ConfigurationHelpers::loadYarpConfigurationString(
    const std::shared_ptr<const sdf::Element>& sdf, yarp::os::Property& config)
{
    if (!sdf->HasElement("yarpConfigurationString"))
    {
        yError() << "Yarp configuration string not found";
        return false;
    }

    std::string yarpConfString = sdf->Get<std::string>("yarpConfigurationString");
    auto wipe = false;
    config.fromString(yarpConfString, wipe);
    yInfo() << "Yarp configuration string loaded: " << yarpConfString;

    return true;
}

bool isURI(const std::string& filepath)
{
    // Regular expression to match the URI pattern
    std::regex uriRegex("^(\\w+):\\/\\/([^\\/]+)?([^?#]+)?(\\?[^#]*)?(#.*)?$");

    // Check if the string matches the URI pattern
    return std::regex_match(filepath, uriRegex);
}

bool ConfigurationHelpers::loadYarpConfigurationFile(const std::string& yarpConfigurationFile,
                                                     yarp::os::Property& config)
{
    std::string filepath{};
    auto sysPaths = gz::common::SystemPaths();

    if (isURI(yarpConfigurationFile))
    {
        yInfo() << "File is a URI: " << yarpConfigurationFile;
        filepath = sysPaths.FindFileURI(yarpConfigurationFile);
        if (filepath.empty())
        {
            yError() << "File not found: " << yarpConfigurationFile;
            return false;
        }
    } else
    {
        if (std::filesystem::exists(yarpConfigurationFile)
            && std::filesystem::is_regular_file(yarpConfigurationFile)
            && std::filesystem::path(yarpConfigurationFile).is_absolute())
        {

            yWarning() << "File is an absolute path: " << yarpConfigurationFile
                       << ". It is recommended to use a URI or relative path.";
            filepath = yarpConfigurationFile;

        } else
        {

            yError() << "File not found: " << yarpConfigurationFile;
            return false;
        }
    }

    if (!config.fromConfigFile(filepath))
    {
        yError() << "Error while loading Yarp configuration file: " << yarpConfigurationFile;
        return false;
    }

    yInfo() << "Yarp configuration file loaded: " << filepath;

    return true;
}

} // namespace gzyarp
