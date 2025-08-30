#include <BaseStateDriver.cpp>
#include <BaseStateShared.hh>
#include <DeviceRegistry.hh>
#include <gzyarp/ConfigurationHelpers.hh>

#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

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
            DeviceRegistry::getHandler()->removeDevice(*m_ecm, m_deviceId);
            m_deviceRegistered = false;
        }

        if (m_baseStateDriver.isValid())
        {
            m_baseStateDriver.close();
        }
    }

    void Configure(const Entity& _entity,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   EntityComponentManager& _ecm,
                   EventManager& /*_eventMgr*/) override
    {

        std::string netWrapper = "multipleanalogsensorsserver";

        gzyarp::PluginConfigureHelper configureHelper(_ecm);

        m_ecm = &_ecm;

        using BaseStateDriverCreator
            = ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::BaseStateDriver>;
        ::yarp::dev::Drivers::factory().add(
            new BaseStateDriverCreator("gazebo_basestate", netWrapper.c_str(), "BaseStateDriver"));

        ::yarp::os::Property driver_properties;

        if (ConfigurationHelpers::loadPluginConfiguration(_sdf, driver_properties))
        {
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
                    << driver_properties.find("yarpDeviceName").asString() << " loaded";
        } else
        {
            yError() << "gz-sim-yarp-basestate-system : missing configuration";
            return;
        }

        std::string yarpDeviceName = driver_properties.find("yarpDeviceName").asString();
        std::string baseLinkName = driver_properties.find("baseLink").asString();

        auto model = Model(_entity);

        Entity baseLinkEntity = model.LinkByName(_ecm, baseLinkName);
        if (_ecm.HasEntity(baseLinkEntity))
        {
            m_baseLinkEntity = baseLinkEntity;
            yInfo() << "gz-sim-yarp-basestate-system: base link name '" << baseLinkName
                    << "' found.";
        } else
        {
            yError() << "gz-sim-yarp-basestate-system: base link name '" << baseLinkName
                     << "' configured was not found in the model definition.";
            return;
        }

        // Create link from entity
        m_baseLink = Link(m_baseLinkEntity);
        // Enable velocity computation in Gazebo
        m_baseLink.EnableVelocityChecks(_ecm, true);
        // Enable acceleration computation in Gazebo
        m_baseLink.EnableAccelerationChecks(_ecm, true);

        m_baseLinkScopedName = scopedName(m_baseLinkEntity, _ecm);
        m_baseStateData.baseLinkScopedName = m_baseLinkScopedName;

        driver_properties.put(YarpBaseStateScopedName.c_str(), m_baseLinkScopedName.c_str());

        driver_properties.put("device", "gazebo_basestate");
        driver_properties.put("robot", m_baseLinkScopedName);

        if (!m_baseStateDriver.open(driver_properties))
        {
            yError() << "gz-sim-yarp-basestate-system Plugin failed: error in opening yarp "
                        "driver";
            return;
        }

        IBaseStateData* iBaseStateData = nullptr;
        auto viewOk = m_baseStateDriver.view(iBaseStateData);

        if (!viewOk || !iBaseStateData)
        {
            yError() << "gz-sim-yarp-basestate-system Plugin failed: error in getting "
                        "IBaseStateData interface";
            return;
        }
        iBaseStateData->setBaseStateData(&m_baseStateData);

        if (!DeviceRegistry::getHandler()
                 ->setDevice(_entity, _ecm, yarpDeviceName, &m_baseStateDriver, m_deviceId))
        {
            yError() << "gz-sim-yarp-basestate-system: failed setting scopedDeviceName(="
                     << m_deviceId << ")";
            return;
        }

        configureHelper.setConfigureIsSuccessful(true);
        m_deviceRegistered = true;
        yInfo() << "Registered YARP device with instance name:" << m_deviceId;
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
        if (dataAvailable && m_baseLink.WorldPose(_ecm).has_value())
        {
            worldBasePose = m_baseLink.WorldPose(_ecm).value();
        } else
        {
            dataAvailable = false;
        }

        // Get the velocity of the origin of the link frame in the world reference frame
        if (dataAvailable && m_baseLink.WorldLinearVelocity(_ecm).has_value())
        {
            worldBaseLinVel = m_baseLink.WorldLinearVelocity(_ecm).value();
        } else
        {
            dataAvailable = false;
        }
        if (dataAvailable && m_baseLink.WorldAngularVelocity(_ecm).has_value())
        {
            worldBaseAngVel = m_baseLink.WorldAngularVelocity(_ecm).value();
        } else
        {
            dataAvailable = false;
        }

        // Get the acceleration of the center of mass of the link in the world reference frame
        if (dataAvailable && m_baseLink.WorldLinearAcceleration(_ecm).has_value())
        {
            worldBaseLinAcc = m_baseLink.WorldLinearAcceleration(_ecm).value();
        } else
        {
            dataAvailable = false;
        }
        if (dataAvailable && m_baseLink.WorldAngularAcceleration(_ecm).has_value())
        {
            worldBaseAngAcc = m_baseLink.WorldAngularAcceleration(_ecm).value();
        } else
        {
            dataAvailable = false;
        }

        {
            std::lock_guard<std::mutex> lock(m_baseStateData.mutex);
            if (!dataAvailable)
            {
                m_baseStateData.dataAvailable = false;
                return;
            }

            m_baseStateData.dataAvailable = true;

            // Write to split buffers
            m_baseStateData.position[0] = worldBasePose.Pos().X();
            m_baseStateData.position[1] = worldBasePose.Pos().Y();
            m_baseStateData.position[2] = worldBasePose.Pos().Z();

            m_baseStateData.orientation[0] = worldBasePose.Rot().Roll();
            m_baseStateData.orientation[1] = worldBasePose.Rot().Pitch();
            m_baseStateData.orientation[2] = worldBasePose.Rot().Yaw();

            m_baseStateData.linVel[0] = worldBaseLinVel.X();
            m_baseStateData.linVel[1] = worldBaseLinVel.Y();
            m_baseStateData.linVel[2] = worldBaseLinVel.Z();

            m_baseStateData.angVel[0] = worldBaseAngVel.X();
            m_baseStateData.angVel[1] = worldBaseAngVel.Y();
            m_baseStateData.angVel[2] = worldBaseAngVel.Z();

            m_baseStateData.linAcc[0] = worldBaseLinAcc.X();
            m_baseStateData.linAcc[1] = worldBaseLinAcc.Y();
            m_baseStateData.linAcc[2] = worldBaseLinAcc.Z();

            m_baseStateData.angAcc[0] = worldBaseAngAcc.X();
            m_baseStateData.angAcc[1] = worldBaseAngAcc.Y();
            m_baseStateData.angAcc[2] = worldBaseAngAcc.Z();

            m_baseStateData.simTimestamp.update(_info.simTime.count() / 1e9);
        }
    }

private:
    bool m_deviceRegistered;
    std::string m_baseLinkScopedName;
    std::string m_deviceId;
    Entity m_baseLinkEntity;
    Link m_baseLink;
    yarp::dev::PolyDriver m_baseStateDriver;
    BaseStateData m_baseStateData;
    yarp::os::Network m_yarpNetwork;
    EntityComponentManager* m_ecm;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::BaseState,
              gz::sim::System,
              gzyarp::BaseState::ISystemConfigure,
              gzyarp::BaseState::ISystemPostUpdate)
