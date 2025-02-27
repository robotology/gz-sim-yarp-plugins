#pragma once

#include <cstdlib>
#include <mutex>
#include <string>
#include <memory>

namespace gzyarp
{

struct DepthCameraData
{
    std::mutex m_mutex;
    int m_width=0;
    int m_height=0;
    int m_imageBufferSize=0;
    int m_depthFrameBufferSize=0;
    std::unique_ptr<unsigned char[]> m_imageBuffer;
    std::unique_ptr<float[]> m_depthFrame_Buffer;
    std::string sensorScopedName="";
    double simTime=0.0;

    void init(const int width, const int height, const std::string& _sensorScopedName)
    {
        this->m_height = height;
        this->m_width = width;
        this->m_imageBufferSize = 3 * this->m_width * this->m_height;
        this->m_imageBuffer = std::make_unique<unsigned char[]>(this->m_imageBufferSize);
        this->m_depthFrameBufferSize = this->m_width * this->m_height;
        this->m_depthFrame_Buffer = std::make_unique<float[]>(this->m_depthFrameBufferSize);
        this->sensorScopedName = _sensorScopedName;
        return;
    }
};

class IDepthCameraData
{
public:
    virtual void setDepthCameraData(DepthCameraData* dataPtr) = 0;

    virtual ~IDepthCameraData() {};
};

} // namespace gzyarp
