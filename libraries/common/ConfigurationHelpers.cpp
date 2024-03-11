#include <ConfigurationHelpers.hh>

#include <gz/common/SystemPaths.hh>

#include <string>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gzyarp
{

bool loadYarpConfigurationFile(const std::string& _filename, yarp::os::Property& _config)
{
    auto sysPaths = gz::common::SystemPaths();
    auto filepath = sysPaths.FindFileURI(_filename);

    if (filepath.empty())
    {
        yError() << "Yarp configuration file not found: " << _filename;
        return false;
    }

    if (!_config.fromConfigFile(filepath))
    {
        yError() << "Error while loading Yarp configuration file: " << _filename;
        return false;
    }

    yInfo() << "Yarp configuration file loaded: " << filepath;

    return true;
}

bool loadYarpConfigurationString(const std::shared_ptr<const sdf::Element>& _sdf,
                                 yarp::os::Property& _config)
{
    if (!_sdf->HasElement("yarpConfigurationString"))
    {
        yError() << "Yarp configuration string not found";
        return false;
    }

    std::string yarpConfString = _sdf->Get<std::string>("yarpConfigurationString");
    auto wipe = false;
    _config.fromString(yarpConfString, wipe);
    yInfo() << "Yarp configuration string loaded: " << yarpConfString;

    return true;
}

bool loadPluginConfiguration(const std::shared_ptr<const sdf::Element>& _sdf,
                             yarp::os::Property& _config)
{
    if (_sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_path = _sdf->Get<std::string>("yarpConfigurationFile");
        return loadYarpConfigurationFile(ini_file_path, _config);
    }

    if (_sdf->HasElement("yarpConfigurationString"))
    {
        return loadYarpConfigurationString(_sdf, _config);
    }

    return false;
}

} // namespace gzyarp
