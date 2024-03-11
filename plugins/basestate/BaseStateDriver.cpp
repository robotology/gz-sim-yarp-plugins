#include <BaseStateDataSingleton.hh>
#include <Handler.hh>

#include <mutex>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{
class BaseStateDriver;
}
} // namespace dev
} // namespace yarp

const std::string YarpBaseStateScopedName = "modelScopedName";
const unsigned YarpBaseStateChannelsNumber = 18; // The BaseState sensor has 18 fixed channels

class yarp::dev::gzyarp::BaseStateDriver : public yarp::dev::DeviceDriver,
                                           public yarp::dev::IAnalogSensor,
                                           public yarp::dev::IPreciselyTimed
{
public:
    BaseStateDriver()
    {
    }
    virtual ~BaseStateDriver()
    {
    }

    // DeviceDriver
    virtual bool open(yarp::os::Searchable& config)
    {

        std::string m_modelScopedName(
            config.find(YarpBaseStateScopedName.c_str()).asString().c_str());

        std::string separator = "/";
        auto pos = config.find("baseLink").asString().find_last_of(separator);
        if (pos == std::string::npos)
        {
            m_baseLinkName = config.find("baseLink").asString();
        } else
        {
            m_baseLinkName = config.find("baseLink").asString().substr(pos + separator.size() - 1);
        }

        m_baseLinkData
            = ::gzyarp::BaseStateDataSingleton::getBaseStateDataHandler()->getBaseStateData(
                m_modelScopedName);

        if (!m_baseLinkData)
        {
            yError() << "Error, BaseState sensor was not found";
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        return true;
    }

    int read(yarp::sig::Vector& out)
    {
        if (out.size() != YarpBaseStateChannelsNumber)
        {
            out.resize(YarpBaseStateChannelsNumber);
        }

        {
            std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);

            if (!m_baseLinkData->dataAvailable)
            {
                yWarning() << "BaseState data not available";
                return AS_ERROR;
            }

            yarp::sig::Vector baseStateData;

            baseStateData.resize(YarpBaseStateChannelsNumber, 0.0);

            for (size_t i = 0; i < YarpBaseStateChannelsNumber; i++)
            {
                baseStateData[i] = m_baseLinkData->data[i];
            }

            out = baseStateData;
        }

        return AS_OK;
    }

    yarp::os::Stamp getLastInputStamp()
    {
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);

        if (!m_baseLinkData->dataAvailable)
        {
            yDebug() << "BaseState data not available.";
        }

        return m_baseLinkData->simTimestamp;
    }

    int calibrateChannel(int ch, double v)
    {
        return AS_OK;
    }

    int calibrateChannel(int ch)
    {
        return AS_OK;
    }

    int calibrateSensor(const yarp::sig::Vector& value)
    {
        return AS_OK;
    }

    int calibrateSensor()
    {
        return AS_OK;
    }

    int getChannels()
    {
        return AS_OK;
    }

    int getState(int ch)
    {
        return AS_OK;
    }

private:
    BaseStateData* m_baseLinkData;
    std::string m_baseLinkName;
};
