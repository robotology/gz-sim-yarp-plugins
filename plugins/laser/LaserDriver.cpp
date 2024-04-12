#include <DeviceRegistry.hh>
#include <LaserShared.hh>

#include <cstddef>
#include <mutex>

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/LaserMeasurementData.h>
#include <yarp/dev/Lidar2DDeviceBase.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/sig/Vector.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{
class LaserDriver;
}
} // namespace dev
} // namespace yarp

const std::string YarpLaserScopedName = "sensorScopedName";

class yarp::dev::gzyarp::LaserDriver : public yarp::dev::Lidar2DDeviceBase,
                                       public yarp::dev::DeviceDriver,
                                       public ::gzyarp::ILaserData
{
public:
    LaserDriver()
    {
    }
    virtual ~LaserDriver()
    {
    }

    // DEVICE DRIVER
    bool open(yarp::os::Searchable& config) override
    {
        std::string sensorScopedName(config.find(YarpLaserScopedName.c_str()).asString().c_str());

        std::string separator = "/";
        auto pos = config.find("sensor_name").asString().find_last_of(separator);
        if (pos == std::string::npos)
        {
            m_sensorName = config.find("sensor_name").asString();
        } else
        {
            m_sensorName = config.find("sensor_name").asString().substr(pos + separator.size() - 1);
        }
        m_frameName = m_sensorName;

        if (this->parseConfiguration(config) == false)
        {
            yError() << "error parsing parameters";
            return false;
        }
        m_device_status = yarp::dev::IRangefinder2D::Device_status::DEVICE_OK_IN_USE;

        return true;
    }

    bool close() override
    {
        return true;
    }

    // Lidar2DDeviceBase
    bool acquireDataFromHW() override
    {
        return true;
    }

    bool getRawData(yarp::sig::Vector& ranges, double* timestamp) override
    {
        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);

        ranges.resize(m_sensorData->m_data.size(), 0.0);
        for (size_t i = 0; i < m_sensorData->m_data.size(); i++)
        {
            ranges[i] = m_sensorData->m_data[i];
        }
        *timestamp = m_sensorData->simTime;

        return true;
    }

    // IRangefinder2D
    bool setDistanceRange(double min, double max) override
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        yError() << "setDistanceRange() Not yet implemented";
        return false;
    }

    bool setScanLimits(double min, double max) override
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        yError() << "setScanLimits() Not yet implemented";
        return false;
    }

    bool setHorizontalResolution(double step) override
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        yError() << "setHorizontalResolution() Not yet implemented";
        return false;
    }

    bool setScanRate(double rate) override
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        yError() << "setScanRate() Not yet implemented";
        return false;
    }

    // ILaserData

    void setLaserData(::gzyarp::LaserData* dataPtr) override
    {
        m_sensorData = dataPtr;
    }

private:
    ::gzyarp::LaserData* m_sensorData;
    std::string m_sensorName;
    std::string m_frameName;
};
