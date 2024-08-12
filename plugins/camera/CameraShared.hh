#pragma once

#include <mutex>
#include <string>

namespace gzyarp
{

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

class ICameraData
{
public:
    virtual void setCameraData(CameraData* dataPtr) = 0;

    virtual ~ICameraData() {};
};

} // namespace gzyarp
