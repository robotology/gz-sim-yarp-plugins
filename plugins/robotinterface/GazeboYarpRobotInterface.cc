#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <yarp/robotinterface/XMLReader.h>
#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/LogStream.h>
#include "../../libraries/singleton-devices/Handler.hh"


using namespace gz;
using namespace sim;
using namespace systems;
 
class GazeboYarpRobotInterface
      : public System,
        public ISystemConfigure
{
    public: 
        GazeboYarpRobotInterface(): m_robotInterfaceCorrectlyStarted(false)
        {
        }
        
        virtual ~GazeboYarpRobotInterface()
        {
            CloseRobotInterface();
            yarp::os::Network::fini();
        }

        void CloseRobotInterface()
        {
            if(m_robotInterfaceCorrectlyStarted) 
            {
                // Close robotinterface
                bool ok = m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt1);
                if (!ok) 
                {
                    yError() << "GazeboYarpRobotInterface: impossible to run phase ActionPhaseInterrupt1 robotinterface";
                }
                ok = m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseShutdown);
                if (!ok) 
                {
                    yError() << "GazeboYarpRobotInterface: impossible  to run phase ActionPhaseShutdown in robotinterface";
                }
                m_robotInterfaceCorrectlyStarted = false;
            }
        }

        virtual void Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager &_ecm,
                                EventManager &/*_eventMgr*/) override
        {
            yarp::os::Network::init();
            if (!yarp::os::Network::checkNetwork())
            {
                yError() << "GazeboYarpRobotInterface : yarp network does not seem to be available, is the yarpserver running?";
                return;
            }
            auto model = Model(_entity);

            bool loaded_configuration = false;
            if (_sdf->HasElement("yarpRobotInterfaceConfigurationFile"))
            {
                robotinterface_file_name = _sdf->Get<std::string>("yarpRobotInterfaceConfigurationFile");
                if (robotinterface_file_name == "") 
                {
                    yError() << "GazeboYarpRobotInterface error: failure in finding robotinterface configuration for model" << model.Name(_ecm) << "\n"
                            << "GazeboYarpRobotInterface error: yarpRobotInterfaceConfigurationFile : " << robotinterface_file_name;
                    loaded_configuration = false;
                } 
                else 
                {
                    m_xmlRobotInterfaceResult = m_xmlRobotInterfaceReader.getRobotFromFile(robotinterface_file_name);
                    if (m_xmlRobotInterfaceResult.parsingIsSuccessful) 
                    {
                        loaded_configuration = true;
                    } 
                    else 
                    {
                        yError() << "GazeboYarpRobotInterface error: failure in parsing robotinterface configuration for model" << model.Name(_ecm) << "\n"
                                << "GazeboYarpRobotInterface error: yarpRobotInterfaceConfigurationFile : " << robotinterface_file_name;
                        loaded_configuration = false;
                    }
                }
            }
            if (!loaded_configuration) 
            {
                yError() << "GazeboYarpRobotInterface : xml file specified in yarpRobotInterfaceConfigurationFile not found or not loaded.";
                return;
            }

            yarp::dev::PolyDriverList externalDriverList;

            Handler::getHandler()->getDevicesAsPolyDriverList(scopedName(model.Entity(), _ecm), externalDriverList, m_deviceScopedNames);

            bool ok = m_xmlRobotInterfaceResult.robot.setExternalDevices(externalDriverList);
            if (!ok) 
            {
                yError() << "GazeboYarpRobotInterface : impossible to set external devices";
                return;
            }
            
            // Start robotinterface
            ok = m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup);
            if (!ok) 
            {
                yError() << "GazeboYarpRobotInterface : impossible to start robotinterface";
                m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt1);
                m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseShutdown);
                return;
            }
            m_robotInterfaceCorrectlyStarted = true;
        
        }

    
    private:
        yarp::robotinterface::XMLReader m_xmlRobotInterfaceReader;
        yarp::robotinterface::XMLReaderResult m_xmlRobotInterfaceResult;
        std::string robotinterface_file_name;
        std::vector<std::string> m_deviceScopedNames;
        bool m_robotInterfaceCorrectlyStarted;

};

// Register plugin
GZ_ADD_PLUGIN(GazeboYarpRobotInterface,
                gz::sim::System,
                GazeboYarpRobotInterface::ISystemConfigure)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(GazeboYarpRobotInterface, "gz::sim::systems::GazeboYarpRobotInterface")