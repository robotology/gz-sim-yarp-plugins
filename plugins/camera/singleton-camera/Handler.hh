#include <gz/common/Event.hh>
#include <mutex>
#include <string>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

struct CameraData
{
    std::mutex m_mutex;
    int m_width;
    int m_height;
    int m_bufferSize;
    unsigned char* m_imageBuffer;
    std::string sensorScopedName;
    double simTime;
};

namespace gzyarp
{

class HandlerCamera
{
public:
    static HandlerCamera* getHandler();

    bool setSensor(CameraData* _sensorDataPtr);

    CameraData* getSensor(const std::string& sensorScopedName) const;

    void removeSensor(const std::string& sensorName);

private:
    HandlerCamera();
    static HandlerCamera* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, CameraData*> SensorsMap;
    SensorsMap m_sensorsMap;
};

} // namespace gzyarp
