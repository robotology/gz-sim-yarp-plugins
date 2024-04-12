#include <DeviceRegistry.hh>
#include <ImuShared.hh>

#include <cstddef>
#include <mutex>
#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
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
class ImuDriver;
}
} // namespace dev
} // namespace yarp

const unsigned YarpIMUChannelsNumber = 9;
constexpr size_t rpyStartIdx = 0;
constexpr size_t accelStartIdx = 3;
constexpr size_t gyroStartIdx = 6;
const std::string YarpIMUScopedName = "sensorScopedName";
/*
 *
 * 0  1  2  = Euler orientation data (Kalman filter processed)
 * 3  4  5  = Calibrated 3-axis (X, Y, Z) acceleration data
 * 6  7  8  = Calibrated 3-axis (X, Y, Z) gyroscope data
 *
 */
class yarp::dev::gzyarp::ImuDriver : public yarp::dev::DeviceDriver,
                                     public yarp::dev::IThreeAxisGyroscopes,
                                     public yarp::dev::IThreeAxisLinearAccelerometers,
                                     public yarp::dev::IOrientationSensors,
                                     public yarp::dev::IThreeAxisMagnetometers,
                                     public ::gzyarp::IImuData
{

public:
    ImuDriver()
    {
    }
    virtual ~ImuDriver()
    {
    }

    // DEVICE DRIVER
    bool open(yarp::os::Searchable& config) override
    {

        std::string sensorScopedName(config.find(YarpIMUScopedName.c_str()).asString().c_str());

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

        return true;
    }

    bool close() override
    {
        return true;
    }

    // THREE AXIS GYROSCOPES
    size_t getNrOfThreeAxisGyroscopes() const override
    {
        return 1;
    }

    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override
    {
        return genericGetStatus(sens_index);
    }

    bool getThreeAxisGyroscopeName(size_t sens_index, std::string& name) const override
    {
        return genericGetSensorName(sens_index, name);
    }

    bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string& frameName) const override
    {
        return genericGetFrameName(sens_index, frameName);
    }

    bool getThreeAxisGyroscopeMeasure(size_t sens_index,
                                      yarp::sig::Vector& out,
                                      double& timestamp) const override
    {
        return genericGetMeasure(sens_index, out, timestamp, gyroStartIdx);
    }

    // THREE AXIS LINEAR ACCELEROMETERS
    size_t getNrOfThreeAxisLinearAccelerometers() const override
    {
        return 1;
    }

    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override
    {
        return genericGetStatus(sens_index);
    }

    bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string& name) const override
    {
        return genericGetSensorName(sens_index, name);
    }

    bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index,
                                                  std::string& frameName) const override
    {
        return genericGetFrameName(sens_index, frameName);
    }

    bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index,
                                                yarp::sig::Vector& out,
                                                double& timestamp) const override
    {
        return genericGetMeasure(sens_index, out, timestamp, accelStartIdx);
    }

    // ORIENTATION SENSORS
    size_t getNrOfOrientationSensors() const override
    {
        return 1;
    }

    yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override
    {
        return genericGetStatus(sens_index);
    }

    bool getOrientationSensorName(size_t sens_index, std::string& name) const override
    {
        return genericGetSensorName(sens_index, name);
    }

    bool getOrientationSensorFrameName(size_t sens_index, std::string& frameName) const override
    {
        return genericGetFrameName(sens_index, frameName);
    }

    bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index,
                                                   yarp::sig::Vector& rpy,
                                                   double& timestamp) const override
    {
        return genericGetMeasure(sens_index, rpy, timestamp, rpyStartIdx);
    }

    // THREE AXIS MAGNETOMETERS

    size_t getNrOfThreeAxisMagnetometers() const override
    {
        return 1;
    }

    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override
    {
        return genericGetStatus(sens_index);
    }

    bool getThreeAxisMagnetometerName(size_t sens_index, std::string& name) const override
    {
        return genericGetSensorName(sens_index, name);
    }

    bool getThreeAxisMagnetometerFrameName(size_t sens_index, std::string& frameName) const override
    {
        return genericGetFrameName(sens_index, frameName);
    }

    bool getThreeAxisMagnetometerMeasure(size_t sens_index,
                                         yarp::sig::Vector& out,
                                         double& timestamp) const override
    {
        return genericGetMeasure(sens_index, out, timestamp, 0);
    }

    // IIMUDATA

    void setImuData(::gzyarp::ImuData* dataPtr) override
    {
        m_sensorData = dataPtr;
    }

private:
    ::gzyarp::ImuData* m_sensorData = nullptr;
    std::string m_sensorName;
    std::string m_frameName;

    yarp::dev::MAS_status genericGetStatus(size_t sens_index) const
    {
        if (sens_index != 0)
        {
            yError() << "IMUDriver: sens_index must be equal to 0, since there is  only one sensor "
                        "in consideration";
            return yarp::dev::MAS_status::MAS_ERROR;
        }

        return yarp::dev::MAS_status::MAS_OK;
    }

    bool genericGetSensorName(size_t sens_index, std::string& name) const
    {
        if (sens_index != 0)
        {
            yError() << "IMUDriver: sens_index must be equal to 0, since there is  only one sensor "
                        "in consideration";
            return false;
        }

        name = m_sensorName;
        return true;
    }

    bool genericGetFrameName(size_t sens_index, std::string& frameName) const
    {
        if (sens_index != 0)
        {
            yError() << "IMUDriver: sens_index must be equal to 0, since there is  only one sensor "
                        "in consideration";
            return false;
        }

        frameName = m_frameName;
        return true;
    }

    bool genericGetMeasure(size_t sens_index,
                           yarp::sig::Vector& out,
                           double& timestamp,
                           size_t startIdx) const
    {

        if (sens_index != 0)
        {
            yError() << "IMUDriver: sens_index must be equal to 0, since there is  only one sensor "
                        "in consideration";
            return false;
        }

        out.resize(3);
        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
        out[0] = m_sensorData->m_data[startIdx];
        out[1] = m_sensorData->m_data[startIdx + 1];
        out[2] = m_sensorData->m_data[startIdx + 2];

        timestamp = m_sensorData->simTime;
        return true;
    }
};
