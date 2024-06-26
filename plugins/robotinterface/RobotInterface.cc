#include <ConfigurationHelpers.hh>
#include <DeviceRegistry.hh>

#include <memory>
#include <sdf/Element.hh>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/robotinterface/Types.h>
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

        if (!loadYarpRobotInterfaceConfigurationFile(_sdf, _ecm, model))
        {
            yError("gz-sim-yarp-robotinterface-system : Error loading robotinterface configuration "
                   "file");
            return;
        }

        yarp::dev::PolyDriverList externalDriverList;

        DeviceRegistry::getHandler()->getDevicesAsPolyDriverList(_ecm,
                                                                 scopedName(model.Entity(), _ecm),
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
    yarp::robotinterface::XMLReaderResult m_xmlRobotInterfaceResult;
    std::vector<std::string> m_deviceScopedNames;
    bool m_robotInterfaceCorrectlyStarted;

    bool loadYarpRobotInterfaceConfigurationFile(const std::shared_ptr<const sdf::Element>& _sdf,
                                                 const EntityComponentManager& _ecm,
                                                 const Model& model)
    {
        if (!_sdf->HasElement("yarpRobotInterfaceConfigurationFile"))
        {
            yError() << "gz-sim-yarp-robotinterface-system :"
                        "yarpRobotInterfaceConfigurationFile element not found";

            return false;
        }

        if (!ConfigurationHelpers::loadRobotInterfaceConfiguration(_sdf, m_xmlRobotInterfaceResult))
        {
            yError() << "gz-sim-yarp-robotinterface-system :"
                        "yarpRobotInterfaceConfigurationFile not found";
            return false;
        }

        return true;
    }
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::RobotInterface, gz::sim::System, gzyarp::RobotInterface::ISystemConfigure)
