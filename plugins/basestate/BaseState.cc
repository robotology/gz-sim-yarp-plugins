#include "BaseStateDriver.cpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{

class BaseState : public System, public ISystemConfigure, public ISystemPostUpdate
{
public:
    BaseState()
        : m_deviceRegistered(false)
    {
    }

    virtual ~BaseState()
    {
        if (m_deviceRegistered)
        {
            Handler::getHandler()->removeDevice(m_deviceScopedName);
            m_deviceRegistered = false;
        }

        if (m_baseStateDriver.isValid())
            m_baseStateDriver.close();
        HandlerBaseState::getHandler()->removeModel(m_modelScopedName);
        yarp::os::Network::fini();
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork())
        {
            yError() << "Yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        std::string netWrapper = "analogServer";
        ::yarp::dev::Drivers::factory().add(
            new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::BaseStateDriver>("gazebo_"
                                                                                   "basestate",
                                                                                   netWrapper
                                                                                       .c_str(),
                                                                                   "BaseStateDrive"
                                                                                   "r"));

        ::yarp::os::Property driver_properties;

        bool wipe = false;
        if (_sdf->HasElement("yarpConfigurationString"))
        {
            std::string configuration_string = _sdf->Get<std::string>("yarpConfigurationString");
            driver_properties.fromString(configuration_string, wipe);
            if (!driver_properties.check("yarpDeviceName"))
            {
                yError() << "gz-sim-yarp-basestate-system : missing yarpDeviceName parameter";
                return;
            }
            if (!driver_properties.check("baseLink"))
            {
                yError() << "gz-sim-yarp-basestate-system : missing baseLink parameter";
                return;
            }
            yInfo() << "gz-sim-yarp-basestate-system: configuration of device "
                    << driver_properties.find("yarpDeviceName").asString()
                    << " loaded from yarpConfigurationString : " << configuration_string << "\n";
        } else
        {
            yError() << "gz-sim-yarp-basestate-system : missing yarpConfigurationString element";
            return;
        }

        std::string deviceName = driver_properties.find("yarpDeviceName").asString();
        std::string baseLinkName = driver_properties.find("baseLink").asString();

        auto model = Model(_entity);

        Entity baseLinkEntity = model.LinkByName(_ecm, baseLinkName);
        if (_ecm.HasEntity(baseLinkEntity))
        {
            this->m_baseLinkEntity = baseLinkEntity;
            yInfo() << "gz-sim-yarp-basestate-system: base link name '" << baseLinkName
                    << "' found.";
        } else
        {
            yError() << "gz-sim-yarp-basestate-system: base link name '" << baseLinkName
                     << "' configured was not found in the model definition.";
            return;
        }

        this->m_baseLink = Link(this->m_baseLinkEntity);
        // Enable velocity computation in Gazebo
        this->m_baseLink.EnableVelocityChecks(_ecm, true);
        // Enable acceleration computation in Gazebo
        this->m_baseLink.EnableAccelerationChecks(_ecm, true);

        m_modelScopedName = scopedName(m_baseLinkEntity, _ecm);
        this->m_baseStateData.m_modelScopedName = m_modelScopedName;

        driver_properties.put(YarpBaseStateScopedName.c_str(), m_modelScopedName.c_str());

        if (!driver_properties.check("yarpDeviceName"))
        {
            yError() << "gz-sim-yarp-basestate-system : missing yarpDeviceName parameter for "
                        "device "
                     << m_modelScopedName;
            return;
        }

        // Insert the pointer in the singleton handler for retrieving it in the yarp driver
        HandlerBaseState::getHandler()->setModel(&(this->m_baseStateData));

        driver_properties.put("device", "gazebo_basestate");
        driver_properties.put("robot", m_modelScopedName);

        if (!m_baseStateDriver.open(driver_properties))
        {
            yError() << "gz-sim-yarp-basestate-system Plugin failed: error in opening yarp "
                        "driver";
            return;
        }

        m_deviceScopedName
            = m_modelScopedName + "/" + driver_properties.find("yarpDeviceName").asString();

        if (!Handler::getHandler()->setDevice(m_deviceScopedName, &m_baseStateDriver))
        {
            yError() << "gz-sim-yarp-basestate-system: failed setting scopedDeviceName(="
                     << m_deviceScopedName << ")";
            return;
        }
        m_deviceRegistered = true;
        yInfo() << "Registered YARP device with instance name:" << m_deviceScopedName;
    }

    virtual void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        bool dataAvailable = true;

        math::Pose3d worldBasePose;
        math::Vector3d worldBaseLinVel;
        math::Vector3d worldBaseAngVel;
        math::Vector3d worldBaseLinAcc;
        math::Vector3d worldBaseAngAcc;

        // Get the pose of the origin of the link frame in the world reference frame
        if (dataAvailable && this->m_baseLink.WorldPose(_ecm).has_value())
            worldBasePose = this->m_baseLink.WorldPose(_ecm).value();
        else
            dataAvailable = false;

        // Get the velocity of the origin of the link frame in the world reference frame
        if (dataAvailable && this->m_baseLink.WorldLinearVelocity(_ecm).has_value())
            worldBaseLinVel = this->m_baseLink.WorldLinearVelocity(_ecm).value();
        else
            dataAvailable = false;
        if (dataAvailable && this->m_baseLink.WorldAngularVelocity(_ecm).has_value())
            worldBaseAngVel = this->m_baseLink.WorldAngularVelocity(_ecm).value();
        else
            dataAvailable = false;

        // Get the acceleration of the center of mass of the link in the world reference frame
        if (dataAvailable && this->m_baseLink.WorldLinearAcceleration(_ecm).has_value())
            worldBaseLinAcc = this->m_baseLink.WorldLinearAcceleration(_ecm).value();
        else
            dataAvailable = false;
        if (dataAvailable && this->m_baseLink.WorldAngularAcceleration(_ecm).has_value())
            worldBaseAngAcc = this->m_baseLink.WorldAngularAcceleration(_ecm).value();
        else
            dataAvailable = false;

        std::lock_guard<std::mutex> lock(m_baseStateData.m_mutex);
        if (!dataAvailable)
        {
            m_baseStateData.m_dataAvailable = false;
            return;
        }

        m_baseStateData.m_dataAvailable = true;

        // Serialize the state vector
        m_baseStateData.m_data[0] = worldBasePose.Pos().X();
        m_baseStateData.m_data[1] = worldBasePose.Pos().Y();
        m_baseStateData.m_data[2] = worldBasePose.Pos().Z();
        m_baseStateData.m_data[3] = worldBasePose.Rot().Roll();
        m_baseStateData.m_data[4] = worldBasePose.Rot().Pitch();
        m_baseStateData.m_data[5] = worldBasePose.Rot().Yaw();

        m_baseStateData.m_data[6] = worldBaseLinVel.X();
        m_baseStateData.m_data[7] = worldBaseLinVel.Y();
        m_baseStateData.m_data[8] = worldBaseLinVel.Z();
        m_baseStateData.m_data[9] = worldBaseAngVel.X();
        m_baseStateData.m_data[10] = worldBaseAngVel.Y();
        m_baseStateData.m_data[11] = worldBaseAngVel.Z();

        m_baseStateData.m_data[12] = worldBaseLinAcc.X();
        m_baseStateData.m_data[13] = worldBaseLinAcc.Y();
        m_baseStateData.m_data[14] = worldBaseLinAcc.Z();
        m_baseStateData.m_data[15] = worldBaseAngAcc.X();
        m_baseStateData.m_data[16] = worldBaseAngAcc.Y();
        m_baseStateData.m_data[17] = worldBaseAngAcc.Z();

        m_baseStateData.m_simTimestamp.update(_info.simTime.count() / 1e9);
    }

private:
    Entity m_baseLinkEntity;
    Link m_baseLink;
    yarp::dev::PolyDriver m_baseStateDriver;
    std::string m_deviceScopedName;
    std::string m_modelScopedName;
    bool m_deviceRegistered;
    BaseStateData m_baseStateData;
    bool m_bsInitialized;
    gz::msgs::Wrench m_bsMsg;
    std::mutex m_msgMutex;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::BaseState,
              gz::sim::System,
              gzyarp::BaseState::ISystemConfigure,
              gzyarp::BaseState::ISystemPostUpdate)
