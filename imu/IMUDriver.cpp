#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IGenericSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <mutex>
#include "singleton/Handler.hh"

namespace yarp {
    namespace dev {
        class GazeboYarpIMUDriver;
    }
}

const unsigned YarpIMUChannelsNumber = 9; 
constexpr size_t rpyStartIdx   = 0;
constexpr size_t accelStartIdx = 3;
constexpr size_t gyroStartIdx  = 6;
const std::string YarpIMUScopedName = "sensorScopedName";
/*
 *
 * 0  1  2  = Euler orientation data (Kalman filter processed)
 * 3  4  5  = Calibrated 3-axis (X, Y, Z) acceleration data
 * 6  7  8  = Calibrated 3-axis (X, Y, Z) gyroscope data
 * 
 */
class yarp::dev::GazeboYarpIMUDriver: 
    public yarp::dev::IGenericSensor,
    public yarp::dev::DeviceDriver,
    public yarp::dev::IThreeAxisGyroscopes,
    public yarp::dev::IThreeAxisLinearAccelerometers,
    public yarp::dev::IOrientationSensors
{
    public:
        GazeboYarpIMUDriver(){}
        virtual ~GazeboYarpIMUDriver(){}

        //DEVICE DRIVER
        virtual bool open(yarp::os::Searchable& config) 
        {

            std::string sensorScopedName(config.find(YarpIMUScopedName.c_str()).asString().c_str());

            std::string separator = "/";
            auto pos = config.find("sensor_name").asString().find_last_of(separator);
            if (pos == std::string::npos) 
            {
                m_sensorName = config.find("sensor_name").asString();
            } 
            else 
            {
                m_sensorName = config.find("sensor_name").asString().substr(pos + separator.size() - 1); 
            }

            m_frameName = m_sensorName;
            m_sensorData = Handler::getHandler()->getSensor(sensorScopedName);
            
            if (!m_sensorData)
            {
                yError() << "Error, IMU sensor was not found";
                return false;
            }

            return true;
        }

        virtual bool close()
        {
            return true;
        }

        //GENERIC SENSOR
        virtual bool read(yarp::sig::Vector& out)
        {
            if (out.size() != YarpIMUChannelsNumber) 
            {
                out.resize(YarpIMUChannelsNumber);
            }

            std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
            out.resize(YarpIMUChannelsNumber, 0.0);
            for (int i = 0; i < 9; i++)
            {
                out[i] = m_sensorData->m_data[i];
            }

            return true;
        }

        virtual bool getChannels(int *nc)
        {
            if (!nc) 
            {
                return false;
            }
            *nc = YarpIMUChannelsNumber;
            return true;
        }
        virtual bool calibrate(int /*ch*/, double /*v*/)
        {
            return true;
        }

        // THREE AXIS GYROSCOPES
        virtual size_t getNrOfThreeAxisGyroscopes() const
        {
            return 1;
        }

        virtual yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const
        {
            return genericGetStatus(sens_index);
        }

        virtual bool getThreeAxisGyroscopeName(size_t sens_index, std::string& name) const
        {
            return genericGetSensorName(sens_index, name);
        }

        virtual bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string& frameName) const
        {
            return genericGetFrameName(sens_index, frameName);
        }

        virtual bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
        {
            return genericGetMeasure(sens_index, out, timestamp, gyroStartIdx);
        }

        // THREE AXIS LINEAR ACCELEROMETERS
        virtual size_t getNrOfThreeAxisLinearAccelerometers() const
        {
            return 1;
        }

        virtual yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
        {
            return genericGetStatus(sens_index);
        }

        virtual bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string& name) const
        {
            return genericGetSensorName(sens_index, name);
        }

        virtual bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string& frameName) const
        {
            return genericGetFrameName(sens_index, frameName);
        }

        virtual bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
        {
            return genericGetMeasure(sens_index, out, timestamp, accelStartIdx);
        }

        // ORIENTATION SENSORS
        virtual size_t getNrOfOrientationSensors() const
        {
            return 1;
        }

        virtual yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const
        {
            return genericGetStatus(sens_index);
        }

        virtual bool getOrientationSensorName(size_t sens_index, std::string& name) const
        {
            return genericGetSensorName(sens_index, name);
        }

        virtual bool getOrientationSensorFrameName(size_t sens_index, std::string& frameName) const
        {
            return genericGetFrameName(sens_index, frameName);
        }

        virtual bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const
        {
            return genericGetMeasure(sens_index, rpy, timestamp, rpyStartIdx);
        }

    private:
        IMUData* m_sensorData;
        std::string m_sensorName;
        std::string m_frameName;

        yarp::dev::MAS_status genericGetStatus(size_t sens_index) const
        {
            if (sens_index != 0)
            {
                yError() << "GazeboYarpIMUDriver: sens_index must be equal to 0, since there is  only one sensor in consideration";
                return yarp::dev::MAS_status::MAS_ERROR;
            }

            return yarp::dev::MAS_status::MAS_OK;
        }

        bool genericGetSensorName(size_t sens_index, std::string& name) const
        {
            if (sens_index != 0)
            {
                yError() << "GazeboYarpIMUDriver: sens_index must be equal to 0, since there is  only one sensor in consideration";
                return false;
            }

            name = m_sensorName;
            return true;
        }

        bool genericGetFrameName(size_t sens_index, std::string& frameName) const
        {
            if (sens_index != 0)
            {
                yError() << "GazeboYarpIMUDriver: sens_index must be equal to 0, since there is  only one sensor in consideration";
                return false;
            }

            frameName = m_frameName;
            return true;
        }

        bool genericGetMeasure(size_t sens_index, yarp::sig::Vector &out, double &timestamp, size_t startIdx) const {

            if (sens_index != 0)
            {
                yError() << "GazeboYarpIMUDriver: sens_index must be equal to 0, since there is  only one sensor in consideration";
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
