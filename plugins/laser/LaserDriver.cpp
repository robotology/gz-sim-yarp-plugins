#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/LaserMeasurementData.h>
#include <yarp/dev/Lidar2DDeviceBase.h>
#include <mutex>
#include <iostream>
#include "singleton-laser/Handler.hh"
#include "../../libraries/singleton-devices/Handler.hh"


namespace yarp {
    namespace dev {
        class GazeboYarpLaserDriver;
    }
}

const std::string YarpLaserScopedName = "sensorScopedName";

class yarp::dev::GazeboYarpLaserDriver: 
    public yarp::dev::Lidar2DDeviceBase,
    public yarp::dev::DeviceDriver
{
    public:
        GazeboYarpLaserDriver(){}
        virtual ~GazeboYarpLaserDriver(){}

        //DEVICE DRIVER
        virtual bool open(yarp::os::Searchable& config) 
        {
            std::string sensorScopedName(config.find(YarpLaserScopedName.c_str()).asString().c_str());

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
            m_sensorData = HandlerLaser::getHandler()->getSensor(sensorScopedName);

            if (!m_sensorData)
            {
                yError() << "Error, Laser sensor was not found";
                return false;
            }
            if (this->parseConfiguration(config) == false)
            {
                yError() << "error parsing parameters";
                return false;
            }

            return true;
        }

        virtual bool close()
        {
            return true;
        }

        //Lidar2DDeviceBase
        bool acquireDataFromHW()
        {
            return true;
        }


        bool getRawData (yarp::sig::Vector& ranges, double *timestamp)
        {
            std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
            
            ranges.resize(m_sensorData->m_data.size(), 0.0);
            for (size_t i=0; i< m_sensorData->m_data.size(); i++)
            {
                ranges[i] = m_sensorData->m_data[i];
            }
            *timestamp = m_sensorData->simTime;

            return true;

        }


        //IRangefinder2D
        virtual bool setDistanceRange (double min, double max)
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            yError() << "setDistanceRange() Not yet implemented";
            return false;
        }

        virtual bool setScanLimits (double min, double max)
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            yError() << "setScanLimits() Not yet implemented";
            return false;
        }

        virtual bool setHorizontalResolution (double step)
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            yError() << "setHorizontalResolution() Not yet implemented";
            return false;
        }

        virtual bool setScanRate (double rate)
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            yError() << "setScanRate() Not yet implemented";
            return false;
        }


    private:
        LaserData* m_sensorData;
        std::string m_sensorName;
        std::string m_frameName;

};
