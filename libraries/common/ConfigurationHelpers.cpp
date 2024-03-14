#include <ConfigurationHelpers.hh>

#include <filesystem>
#include <regex>
#include <string>

#include <gz/common/SystemPaths.hh>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/robotinterface/XMLReader.h>

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

bool ConfigurationHelpers::loadRobotInterfaceConfiguration(
    const std::shared_ptr<const sdf::Element>& sdf, yarp::robotinterface::XMLReaderResult& result)
{
    if (!sdf->HasElement("yarpRobotInterfaceConfigurationFile"))
    {
        yError() << "yarpRobotInterfaceConfigurationFile element not found";
        return false;
    }

    auto robotinterface_file_name = sdf->Get<std::string>("yarpRobotInterfaceConfigurationFil"
                                                          "e");

    std::string filepath;
    if (!findFile(robotinterface_file_name, filepath))
    {
        yError() << "Error while finding yarpRobotInterfaceConfigurationFile: "
                 << robotinterface_file_name;
        return false;
    }

    yarp::robotinterface::XMLReader xmlRobotInterfaceReader;
    result = xmlRobotInterfaceReader.getRobotFromFile(filepath);

    if (!result.parsingIsSuccessful)
    {
        yError() << "Failure in parsing yarpRobotInterfaceConfigurationFile: "
                 << robotinterface_file_name;
        return false;
    }

    return true;
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

bool ConfigurationHelpers::isURI(const std::string& filepath)
{
    // Regular expression to match the URI pattern
    std::regex uriRegex("^(\\w+):\\/\\/([^\\/]+)?([^?#]+)?(\\?[^#]*)?(#.*)?$");

    // Check if the string matches the URI pattern
    return std::regex_match(filepath, uriRegex);
}

bool ConfigurationHelpers::loadYarpConfigurationFile(const std::string& yarpConfigurationFile,
                                                     yarp::os::Property& config)
{
    std::string filepath;
    if (!findFile(yarpConfigurationFile, filepath))
    {
        yError() << "Error while finding Yarp configuration file: " << yarpConfigurationFile;
        return false;
    }

    if (!config.fromConfigFile(filepath))
    {
        yError() << "Error while loading Yarp configuration file: " << yarpConfigurationFile;
        return false;
    }

    yInfo() << "Yarp configuration file loaded: " << filepath;

    return true;
}

bool ConfigurationHelpers::findFile(const std::string& filename, std::string& filepath)
{
    auto sysPaths = gz::common::SystemPaths();

    if (isURI(filename))
    {
        yInfo() << "File is a URI: " << filename;
        filepath = sysPaths.FindFileURI(filename);
        if (filepath.empty())
        {
            yError() << "File not found: " << filename;
            return false;
        }
    } else
    {
        if (std::filesystem::exists(filename) && std::filesystem::is_regular_file(filename))
        {
            if (std::filesystem::path(filename).is_absolute())
            {
                yWarning() << "File specified with an absolute path: " << filename
                           << ". It is recommended to use a URI.";
                filepath = filename;
            } else
            {
                std::filesystem::path relativePath(filename);
                filepath = std::filesystem::absolute(relativePath).string();
                yWarning() << "File specified with a relative path: " << filename
                           << ", resolved to: " << filepath << ". It is recommended to use a URI.";
            }
        } else
        {

            yError() << "File not found: " << filename;
            return false;
        }
    }

    return true;
}

} // namespace gzyarp
