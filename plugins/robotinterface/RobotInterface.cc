#include "../../libraries/singleton-devices/Handler.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/robotinterface/XMLReader.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{

class RobotInterface : public System, public ISystemConfigure
{
public:
    RobotInterface()
        : m_robotInterfaceCorrectlyStarted(false)
    {
    }

    virtual ~RobotInterface()
    {
        CloseRobotInterface();
        yarp::os::Network::fini();
    }

    void CloseRobotInterface()
    {
        if (m_robotInterfaceCorrectlyStarted)
        {
            // Close robotinterface
            bool ok = m_xmlRobotInterfaceResult.robot.enterPhase(
                yarp::robotinterface::ActionPhaseInterrupt1);
            if (!ok)
            {
                yError() << "gz-sim-yarp-robotinterface-system: impossible to run phase "
                            "ActionPhaseInterrupt1 robotinterface";
            }
            ok = m_xmlRobotInterfaceResult.robot.enterPhase(
                yarp::robotinterface::ActionPhaseShutdown);
            if (!ok)
            {
                yError() << "gz-sim-yarp-robotinterface-system: impossible  to run phase "
                            "ActionPhaseShutdown in robotinterface";
            }
            m_robotInterfaceCorrectlyStarted = false;
        }
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork())
        {
            yError() << "gz-sim-yarp-robotinterface-system : yarp network does not seem to be "
                        "available, is the yarpserver running?";
            return;
        }
        auto model = Model(_entity);

        bool loaded_configuration = false;
        if (_sdf->HasElement("yarpRobotInterfaceConfigurationFile"))
        {
            robotinterface_file_name = _sdf->Get<std::string>("yarpRobotInterfaceConfigurationFil"
                                                              "e");
            if (robotinterface_file_name == "")
            {
                yError() << "gz-sim-yarp-robotinterface-system error: failure in finding "
                            "robotinterface configuration for model"
                         << model.Name(_ecm) << "\n"
                         << "gz-sim-yarp-robotinterface-system error: "
                            "yarpRobotInterfaceConfigurationFile : "
                         << robotinterface_file_name;
                loaded_configuration = false;
            } else
            {
                m_xmlRobotInterfaceResult
                    = m_xmlRobotInterfaceReader.getRobotFromFile(robotinterface_file_name);
                if (m_xmlRobotInterfaceResult.parsingIsSuccessful)
                {
                    loaded_configuration = true;
                } else
                {
                    yError() << "gz-sim-yarp-robotinterface-system error: failure in parsing "
                                "robotinterface configuration for model"
                             << model.Name(_ecm) << "\n"
                             << "gz-sim-yarp-robotinterface-system error: "
                                "yarpRobotInterfaceConfigurationFile : "
                             << robotinterface_file_name;
                    loaded_configuration = false;
                }
            }
        }
        if (!loaded_configuration)
        {
            yError() << "gz-sim-yarp-robotinterface-system : xml file specified in "
                        "yarpRobotInterfaceConfigurationFile not found or not loaded.";
            return;
        }

        yarp::dev::PolyDriverList externalDriverList;

        Handler::getHandler()->getDevicesAsPolyDriverList(scopedName(model.Entity(), _ecm),
                                                          externalDriverList,
                                                          m_deviceScopedNames);

        bool ok = m_xmlRobotInterfaceResult.robot.setExternalDevices(externalDriverList);
        if (!ok)
        {
            yError() << "gz-sim-yarp-robotinterface-system : impossible to set external devices";
            return;
        }

        // Start robotinterface
        ok = m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup);
        if (!ok)
        {
            yError() << "gz-sim-yarp-robotinterface-system : impossible to start robotinterface";
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

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::RobotInterface, gz::sim::System, gzyarp::RobotInterface::ISystemConfigure)
