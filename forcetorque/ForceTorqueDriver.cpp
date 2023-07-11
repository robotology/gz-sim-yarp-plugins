#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <mutex>
#include "singleton/Handler.hh"

namespace yarp {
    namespace dev {
        class GazeboYarpForceTorqueDriver;
    }
}

const unsigned YarpForceTorqueChannelsNumber = 6; //The ForceTorque sensor has 6 fixed channels
const std::string YarpForceTorqueScopedName = "sensorScopedName";

class yarp::dev::GazeboYarpForceTorqueDriver: 
    public yarp::dev::IAnalogSensor,
    public yarp::dev::DeviceDriver,
    public yarp::dev::ISixAxisForceTorqueSensors
{
    public:
        GazeboYarpForceTorqueDriver(){}
        virtual ~GazeboYarpForceTorqueDriver(){}

        //DEVICE DRIVER
        virtual bool open(yarp::os::Searchable& config)
        {

            std::string sensorScopedName(config.find(YarpForceTorqueScopedName.c_str()).asString().c_str());

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
                yError() << "Error, ForceTorque sensor was not found";
                return AS_ERROR;
            }

            return true;
        }

        virtual bool close()
        {
            return true;
        }

        //ANALOG SENSOR
        virtual int read(yarp::sig::Vector& out)
        {
            if (out.size() != YarpForceTorqueChannelsNumber) 
            {
                out.resize(YarpForceTorqueChannelsNumber);
            }

            std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
            yarp::sig::Vector m_forceTorqueData;
            m_forceTorqueData.resize(YarpForceTorqueChannelsNumber, 0.0);
            m_forceTorqueData[0] = m_sensorData->m_data[0];
            m_forceTorqueData[1] = m_sensorData->m_data[1];
            m_forceTorqueData[2] = m_sensorData->m_data[2];
            m_forceTorqueData[3] = m_sensorData->m_data[3];
            m_forceTorqueData[4] = m_sensorData->m_data[4];
            m_forceTorqueData[5] = m_sensorData->m_data[5];

            return AS_OK;
        }

        virtual int getState(int /*channel*/)
        {
            return AS_OK;
        }
        virtual int getChannels()
        {
            return YarpForceTorqueChannelsNumber;
        }
        virtual int calibrateChannel(int /*channel*/, double /*v*/)
        {
            return AS_OK;
        }
        virtual int calibrateSensor()
        {
            return AS_OK;
        }
        virtual int calibrateSensor(const yarp::sig::Vector& /*value*/)
        {
            return AS_OK;
        }
        virtual int calibrateChannel(int /*channel*/)
        {
            return AS_OK;
        }

        // SIX AXIS FORCE TORQUE SENSORS
        virtual size_t getNrOfSixAxisForceTorqueSensors() const 
        {
            return 1;
        }
        virtual yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sens_index) const 
        {
            if (sens_index >= 1)
            {
                return MAS_UNKNOWN;
            }

            return MAS_OK;            
        }
        virtual bool getSixAxisForceTorqueSensorName(size_t sens_index, std::string &name) const 
        {
            if (sens_index >= 1)
            {
                return false;
            }

            name = m_sensorName;
            return true;
        }
        virtual bool getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string &frameName) const 
        {
            if (sens_index >= 1)
            {
                return false;
            }

            frameName = m_frameName;
            return true;            
        }
        virtual bool getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const 
        {
            if (sens_index >= 1)
            {
                return false;
            }

            if (out.size() != YarpForceTorqueChannelsNumber) 
            {
                out.resize(YarpForceTorqueChannelsNumber);
            }
            std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
            yarp::sig::Vector m_forceTorqueData;
            
            m_forceTorqueData.resize(YarpForceTorqueChannelsNumber, 0.0);
            m_forceTorqueData[0] = m_sensorData->m_data[0];
            m_forceTorqueData[1] = m_sensorData->m_data[1];
            m_forceTorqueData[2] = m_sensorData->m_data[2];
            m_forceTorqueData[3] = m_sensorData->m_data[3];
            m_forceTorqueData[4] = m_sensorData->m_data[4];
            m_forceTorqueData[5] = m_sensorData->m_data[5];
            out = m_forceTorqueData;
            
            timestamp = m_sensorData->simTime;
            return true;
        }

    private:
        ForceTorqueData* m_sensorData;
        std::string m_sensorName;
        std::string m_frameName;
};
