#include <boost/bind/bind.hpp>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <gz/plugin.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EventManager.hh>
#include<gz/sim/Types.hh>

#include <mutex>
#include "Handler.hh"

namespace yarp {
    namespace dev {
        class GazeboYarpForceTorqueDriver;
    }
}

const unsigned YarpForceTorqueChannelsNumber = 6; //The ForceTorque sensor has 6 fixed channels
const std::string YarpForceTorqueScopedName = "sensorScopedName";

class yarp::dev::GazeboYarpForceTorqueDriver: 
    public yarp::dev::IAnalogSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver,
    public yarp::dev::ISixAxisForceTorqueSensors
{
    public:
        GazeboYarpForceTorqueDriver(){}
        virtual ~GazeboYarpForceTorqueDriver(){}

       void onUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) {
            auto joint = m_parentSensor.Parent(_ecm).value();
            auto jointWrench = _ecm.Component<components::JointTransmittedWrench>(joint);
            if (nullptr == jointWrench)
            {
                return;
            }
            auto X_JS = _ecm.Component<components::Pose>(m_parentSensor.Entity())->Data();
            math::Vector3d force = X_JS.Rot().Inverse() * msgs::Convert(jointWrench->Data().force());
            math::Vector3d torque = X_JS.Rot().Inverse() * msgs::Convert(jointWrench->Data().torque()) - X_JS.Pos().Cross(force);
            //m_lastTimestamp.update(this->m_parentSensor->LastUpdateTime().Double());
            m_lastTimestamp.update(_info.simTime.count()/1e9);

            std::lock_guard<std::mutex> lock(m_dataMutex);

            for (unsigned i = 0; i < 3; i++) {
                m_forceTorqueData[0 + i] = force[i];
            }

            for (unsigned i = 0; i < 3; i++) {
                m_forceTorqueData[3 + i] = torque[i];
            }
            std::cout << "Force:  " << force << std::endl;
            std::cout << "Torque: " << torque << std::endl << std::endl;
            return;
        }

        /**
         * Yarp interfaces start here
         */

        //DEVICE DRIVER
        virtual bool open(yarp::os::Searchable& config){
            {
                std::lock_guard<std::mutex> lock(m_dataMutex);
                m_forceTorqueData.resize(YarpForceTorqueChannelsNumber, 0.0);
            }

            //Get gazebo pointers
            std::string sensorScopedName(config.find(YarpForceTorqueScopedName.c_str()).asString().c_str());

            std::string separator = "/";
            auto pos = config.find("sensor_name").asString().find_last_of(separator);
            if (pos == std::string::npos) {
                m_sensorName = config.find("sensor_name").asString();
            } 
            else {
                m_sensorName = config.find("sensor_name").asString().substr(pos + separator.size() - 1); 
            }
            m_frameName = m_sensorName;
            m_parentSensor = Handler::getHandler()->getSensor(sensorScopedName);
            /*
            if (!m_parentSensor)
            {
                yError() << "Error, ForceTorque sensor was not found";
                return AS_ERROR;
            }
            */
            //Connect the driver to the gazebo simulation
            //using namespace boost::placeholders;
            //this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpForceTorqueDriver::onUpdate, this, _1));
            return true;
        }
        virtual bool close(){
            //this->m_updateConnection.reset();
            return true;
        }

        //ANALOG SENSOR
        virtual int read(yarp::sig::Vector& out){
            if (m_forceTorqueData.size() != YarpForceTorqueChannelsNumber) {
                return AS_ERROR;
            }

            if (out.size() != YarpForceTorqueChannelsNumber) {
                out.resize(YarpForceTorqueChannelsNumber);
            }

            std::lock_guard<std::mutex> lock(m_dataMutex);
            out = m_forceTorqueData;

            return AS_OK;
        }

        virtual int getState(int /*channel*/){
            return AS_OK;
        }
        virtual int getChannels(){
            return YarpForceTorqueChannelsNumber;
        }
        virtual int calibrateChannel(int /*channel*/, double /*v*/){
            return AS_OK;
        }
        virtual int calibrateSensor(){
            return AS_OK;
        }
        virtual int calibrateSensor(const yarp::sig::Vector& /*value*/){
            return AS_OK;
        }
        virtual int calibrateChannel(int /*channel*/){
            return AS_OK;
        }

        // SIX AXIS FORCE TORQUE SENSORS
        virtual size_t getNrOfSixAxisForceTorqueSensors() const {
            return 1;
        }
        virtual yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sens_index) const {
            if (sens_index >= 1)
            {
                return MAS_UNKNOWN;
            }

            return MAS_OK;            
        }
        virtual bool getSixAxisForceTorqueSensorName(size_t sens_index, std::string &name) const {
            if (sens_index >= 1)
            {
                return false;
            }

            name = m_sensorName;
            return true;
        }
        virtual bool getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string &frameName) const {
            if (sens_index >= 1)
            {
                return false;
            }

            frameName = m_frameName;
            return true;            
        }
        virtual bool getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const {
            if (sens_index >= 1)
            {
                return false;
            }

            if (m_forceTorqueData.size() != YarpForceTorqueChannelsNumber) {
                return false;
            }

            if (out.size() != YarpForceTorqueChannelsNumber) {
                out.resize(YarpForceTorqueChannelsNumber);
            }

            std::lock_guard<std::mutex> lock(m_dataMutex);
            out = m_forceTorqueData;
            timestamp = m_lastTimestamp.getTime();

            return true;
        }

        //PRECISELY TIMED
        virtual yarp::os::Stamp getLastInputStamp() {
            return m_lastTimestamp;
        }


    private:
        yarp::sig::Vector m_forceTorqueData; //buffer for forcetorque sensor data
        yarp::os::Stamp m_lastTimestamp; //buffer for last timestamp data
        mutable std::mutex m_dataMutex; //mutex for accessing the data
        gz::sim::Sensor m_parentSensor;
        //gazebo::event::ConnectionPtr m_updateConnection;
        std::string m_sensorName;
        std::string m_frameName;
        gz::common::ConnectionPtr m_updateConnection;

};
