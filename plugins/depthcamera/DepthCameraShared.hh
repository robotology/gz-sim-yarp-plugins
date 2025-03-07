#pragma once

#include <cstdlib>
#include <mutex>
#include <string>
#include <memory>
#include <yarp/sig/Image.h>
#include <gz/msgs/details/image.pb.h>

namespace gzyarp
{
static const std::map<::gz::msgs::PixelFormatType, YarpVocabPixelTypesEnum> m_format2VocabPixel = {
                                                                                                 {gz::msgs::L_INT8,      VOCAB_PIXEL_MONO},
                                                                                                 {gz::msgs::L_INT16,     VOCAB_PIXEL_MONO16},
                                                                                                 {gz::msgs::RGB_INT8,    VOCAB_PIXEL_RGB},
                                                                                                 {gz::msgs::RGBA_INT8,   VOCAB_PIXEL_RGBA},
                                                                                                 {gz::msgs::BGRA_INT8,   VOCAB_PIXEL_BGRA},
                                                                                                 {gz::msgs::RGB_INT16,   VOCAB_PIXEL_INVALID},
                                                                                                 {gz::msgs::RGB_INT32,   VOCAB_PIXEL_RGB_INT},
                                                                                                 {gz::msgs::BGR_INT8,    VOCAB_PIXEL_BGR},
                                                                                                 {gz::msgs::BGR_INT16,   VOCAB_PIXEL_INVALID},
                                                                                                 {gz::msgs::BGR_INT32,   VOCAB_PIXEL_INVALID},
                                                                                                 {gz::msgs::R_FLOAT16,   VOCAB_PIXEL_MONO_FLOAT},
                                                                                                 {gz::msgs::RGB_FLOAT16, VOCAB_PIXEL_RGB_FLOAT},
                                                                                                 {gz::msgs::R_FLOAT32,   VOCAB_PIXEL_MONO_FLOAT},
                                                                                                 {gz::msgs::RGB_FLOAT32, VOCAB_PIXEL_RGB_FLOAT},
                                                                                                 {gz::msgs::BAYER_RGGB8, VOCAB_PIXEL_ENCODING_BAYER_RGGB8},
                                                                                                 {gz::msgs::BAYER_BGGR8, VOCAB_PIXEL_ENCODING_BAYER_BGGR8},
                                                                                                 {gz::msgs::BAYER_GBRG8, VOCAB_PIXEL_ENCODING_BAYER_GBRG8},
                                                                                                 {gz::msgs::BAYER_GRBG8, VOCAB_PIXEL_ENCODING_BAYER_GRBG8}};

struct DistortionModel
{
    double k1{0.0};
    double k2{0.0};
    double k3{0.0};
    double p1{0.0};
    double p2{0.0};
    double cx{0.0};
    double cy{0.0};
};

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
    double horizontal_fov{0.0};
    double vertical_fov{0.0};
    double nearPlane{0.0};
    double farPlane{0.0};
    double focalLengthX{0.0};
    double focalLengthY{0.0};
    YarpVocabPixelTypesEnum m_imageFormat{VOCAB_PIXEL_RGB};
    YarpVocabPixelTypesEnum m_depthFormat{VOCAB_PIXEL_MONO_FLOAT};
    DistortionModel m_distModel;

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
