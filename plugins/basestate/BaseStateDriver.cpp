#include <BaseStateShared.hh>
#include <DeviceRegistry.hh>

#include <cstddef>
#include <mutex>
#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

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
                                           public yarp::dev::IPreciselyTimed,
                                           public ::gzyarp::IBaseStateData
{
public:
    BaseStateDriver()
    {
    }
    virtual ~BaseStateDriver()
    {
    }

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override
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

        if (!m_baseLinkData)
        {
            yError() << "Error, BaseState sensor was not found";
            return false;
        }

        return true;
    }

    bool close() override
    {
        return true;
    }

    int read(yarp::sig::Vector& out) override
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

    yarp::os::Stamp getLastInputStamp() override
    {
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);

        if (!m_baseLinkData->dataAvailable)
        {
            yDebug() << "BaseState data not available.";
        }

        return m_baseLinkData->simTimestamp;
    }

    int calibrateChannel(int ch, double v) override
    {
        return AS_OK;
    }

    int calibrateChannel(int ch) override
    {
        return AS_OK;
    }

    int calibrateSensor(const yarp::sig::Vector& value) override
    {
        return AS_OK;
    }

    int calibrateSensor() override
    {
        return AS_OK;
    }

    int getChannels() override
    {
        return AS_OK;
    }

    int getState(int ch) override
    {
        return AS_OK;
    }

    // IBaseStateData

    void setBaseStateData(::gzyarp::BaseStateData* baseLinkData) override
    {
        m_baseLinkData = baseLinkData;
    }

private:
    ::gzyarp::BaseStateData* m_baseLinkData;
    std::string m_baseLinkName;
};
