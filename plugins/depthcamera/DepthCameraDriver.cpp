#include <DepthCameraShared.hh>
#include <DeviceRegistry.hh>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/sig/Image.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{
class DepthCameraDriver;
}
} // namespace dev
} // namespace yarp

const std::string YarpDepthCameraScopedName = "sensorScopedName";

class yarp::dev::gzyarp::DepthCameraDriver : public yarp::dev::DeviceDriver,
                                             public yarp::dev::IRGBDSensor,
                                             public ::gzyarp::IDepthCameraData
{
public:
    DepthCameraDriver()
    {
        m_vertical_flip = false;
        m_horizontal_flip = false;
        m_display_time_box = false;
        m_display_timestamp = false;
        counter = 0;
    }

    virtual ~DepthCameraDriver()
    {
    }

    // DEVICE DRIVER
    bool open(yarp::os::Searchable& config) override
    {
        std::string sensorScopedName(config.find(YarpDepthCameraScopedName.c_str()).asString().c_str());

        if (config.check("vertical_flip"))
            m_vertical_flip = true;
        if (config.check("horizontal_flip"))
            m_horizontal_flip = true;
        if (config.check("display_timestamp"))
            m_display_timestamp = true;
        if (config.check("display_time_box"))
            m_display_time_box = true;

        return true;
    }

    bool close() override
    {
        return true;
    }

    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& _image) override
    {
        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
        _image.resize(width(), height());

        unsigned char* pBuffer = _image.getRawImage();

        if (m_vertical_flip == true && m_horizontal_flip == false)
        {
            int r = 0;
            int c = 0;
            for (int c = 0; c < width(); c++)
            {
                for (int r = 0; r < height(); r++)
                {
                    unsigned char* pixel = _image.getPixelAddress(c, height() - r - 1);
                    pixel[0] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 0);
                    pixel[1] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 1);
                    pixel[2] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 2);
                }
            }
        } else if (m_vertical_flip == false && m_horizontal_flip == true)
        {
            int r = 0;
            int c = 0;
            for (int c = 0; c < width(); c++)
            {
                for (int r = 0; r < height(); r++)
                {
                    unsigned char* pixel = _image.getPixelAddress(width() - c - 1, r);
                    pixel[0] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 0);
                    pixel[1] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 1);
                    pixel[2] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 2);
                }
            }
        } else if (m_vertical_flip == true && m_horizontal_flip == true)
        {
            int r = 0;
            int c = 0;
            for (int c = 0; c < width(); c++)
            {
                for (int r = 0; r < height(); r++)
                {
                    unsigned char* pixel
                        = _image.getPixelAddress(width() - c - 1, height() - r - 1);
                    pixel[0] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 0);
                    pixel[1] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 1);
                    pixel[2] = *(m_sensorData->m_imageBuffer + r * width() * 3 + c * 3 + 2);
                }
            }
        } else
        {
            memcpy(pBuffer, m_sensorData->m_imageBuffer, getRawBufferSize());
        }

        if (m_display_time_box)
        {
            counter++;
            if (counter == 10)
                counter = 0;

            for (int c = 0 + counter * 30; c < 30 + counter * 30; c++)
            {
                for (int r = 0; r < 30; r++)
                {
                    if (counter % 2 == 0)
                    {
                        unsigned char* pixel
                            = _image.getPixelAddress(width() - c - 1, height() - r - 1);
                        pixel[0] = 255;
                        pixel[1] = 0;
                        pixel[2] = 0;
                    } else
                    {
                        unsigned char* pixel
                            = _image.getPixelAddress(width() - c - 1, height() - r - 1);
                        pixel[0] = 0;
                        pixel[1] = 255;
                        pixel[2] = 0;
                    }
                }
            }
        }

        if (m_display_timestamp)
        {
            char txtbuf[1000];
            sprintf(txtbuf, "%.3f", m_sensorData->simTime);
            int len = strlen(txtbuf);
            if (len < 20)
                print(pBuffer, width(), height(), 0, 0, txtbuf, len);
        }

        return true;
    }

    int getRgbHeight() const override
    {
        return m_sensorData->m_height;
    }

    int getRgbWidth() const override
    {
        return m_sensorData->m_width;
    }


    bool setRgbResolution(int width, int height) override
    {
        m_depthCameraSensorPtr->DepthCamera()->SetImageHeight(height);
        m_depthCameraSensorPtr->DepthCamera()->SetImageWidth(width);
    
        m_width  = width;
        m_height = height;
        return true;
    }
    
    bool getRgbFOV(double& horizontalFov, double& verticalFov) const override
    {
        horizontalFov = m_depthCameraSensorPtr->DepthCamera()->HFOV().Degree();
        verticalFov   = m_depthCameraSensorPtr->DepthCamera()->VFOV().Degree();
        return true;
    }
    bool setRgbFOV(double horizontalFov, double verticalFov) override
    {
        ignition::math::Angle hFov;
        hFov.Degree(horizontalFov);
        m_depthCameraSensorPtr->DepthCamera()->SetHFOV(hFov);
        yCWarning(GAZEBODEPTH) << "GazeboDepthCameraDriver: only horizontal fov set!";
        return true;
    }
    bool getRgbMirroring(bool& mirror) const override
    {
        mirror = false;
        return true;
    }
    bool setRgbMirroring(bool mirror) override
    {
        yCError(GAZEBODEPTH)  << "setRgbMirroring not implemented yet";
        return false;
    }
    bool getRgbIntrinsicParam(Property& intrinsic) const override
    {
        return getDepthIntrinsicParam(intrinsic);
    }
    bool getRgbImage(FlexImage& rgbImage, Stamp* timeStamp) const override
    {
        if(!timeStamp)
        {
            yCError(GAZEBODEPTH) << "timestamp pointer invalid!";
            return false;
        }
    
        std::lock_guard<std::mutex> lock(m_colorFrameMutex);
    
        if(m_width == 0 || m_height == 0)
        {
            yCError(GAZEBODEPTH)  << "gazebo returned an invalid image size";
            return false;
        }
        rgbImage.setPixelCode(m_imageFormat);
        rgbImage.resize(m_width, m_height);
        memcpy(rgbImage.getRawImage(), m_imageFrame_Buffer, m_imageFrame_BufferSize);
        timeStamp->update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());
    
        return true;
    }
    
    int getDepthHeight() const override
    {
        return m_height;
    }
    int getDepthWidth() const override
    {
        return m_width;
    }
    bool setDepthResolution(int width, int height) const override
    {
        return setRgbResolution(width, height);
    }
    bool getDepthFOV(double& horizontalFov, double& verticalFov) const override
    {
        return getRgbFOV(horizontalFov, verticalFov);
    }
    bool setDepthFOV(double horizontalFov, double verticalFov) override
    {
        return getRgbFOV(horizontalFov, verticalFov);
    }
    bool getDepthIntrinsicParam(Property& intrinsic) override
    {
        using namespace gazebo::rendering;
    
        Distortion*  distModel;
        DepthCamera* camPtr;
        Value        rectM;
    
        intrinsic.put("physFocalLength", 0.0);
        camPtr = m_depthCameraSensorPtr->DepthCamera().get();
        if(camPtr)
        {
            intrinsic.put("focalLengthX",    1. / camPtr->OgreCamera()->getPixelDisplayRatio());
            intrinsic.put("focalLengthY",    1. / camPtr->OgreCamera()->getPixelDisplayRatio());
            distModel = camPtr->LensDistortion().get();
            if(distModel)
            {
                intrinsic.put("k1",              distModel->K1());
                intrinsic.put("k2",              distModel->K2());
                intrinsic.put("k3",              distModel->K3());
                intrinsic.put("t1",              distModel->P1());
                intrinsic.put("t2",              distModel->P2());
                intrinsic.put("principalPointX", distModel->Center().X());
                intrinsic.put("principalPointY", distModel->Center().Y());
            }
            else
            {
                intrinsic.put("k1",              0.0);
                intrinsic.put("k2",              0.0);
                intrinsic.put("k3",              0.0);
                intrinsic.put("t1",              0.0);
                intrinsic.put("t2",              0.0);
                intrinsic.put("principalPointX", m_width/2.0);
                intrinsic.put("principalPointY", m_height/2.0);
            }
    
        }
        intrinsic.put("rectificationMatrix", rectM.makeList("1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0"));
        intrinsic.put("distortionModel", "plumb_bob");
        intrinsic.put("stamp", m_colorTimestamp.getTime());
        return true;
    }
    
    double getDepthAccuracy() const override
    {
        return 0.00001;
    }
    
    bool setDepthAccuracy(double accuracy) override
    {
        yCError(GAZEBODEPTH)  << "impossible to set accuracy";
        return false;
    }
    
    bool getDepthClipPlanes(double& nearPlane, double& farPlane) const override
    {
        nearPlane = m_depthCameraSensorPtr->DepthCamera()->NearClip();
        farPlane  = m_depthCameraSensorPtr->DepthCamera()->FarClip();
        return true;
    }
    bool setDepthClipPlanes(double nearPlane, double farPlane) override
    {
        m_depthCameraSensorPtr->DepthCamera()->SetClipDist(nearPlane,farPlane);
        return true;
    }
    bool getDepthMirroring(bool& mirror) const override
    {
        return getRgbMirroring(mirror);
    }
    bool setDepthMirroring(bool mirror) override
    {
        return setRgbMirroring(mirror);
    }
    bool getDepthImage(depthImageType& depthImage, Stamp* timeStamp) const override
    {
        if(!timeStamp)
        {
            yCError(GAZEBODEPTH)  << "gazeboDepthCameraDriver: timestamp pointer invalid!";
            return false;
        }
    
        std::lock_guard<std::mutex> lock(m_depthFrameMutex);
    
        if(m_width == 0 || m_height == 0)
        {
            yCError(GAZEBODEPTH)  << "gazebo returned an invalid image size";
            return false;
        }
    
        depthImage.resize(m_width, m_height);
        //depthImage.setPixelCode(m_depthFormat);
        if(!m_depthQuantizationEnabled) {
            memcpy(depthImage.getRawImage(), m_depthFrame_Buffer, m_width * m_height * sizeof(float));
        }
        else {
            float nearPlane = (float) m_depthCameraSensorPtr->DepthCamera()->NearClip();
            float farPlane = (float) m_depthCameraSensorPtr->DepthCamera()->FarClip();
    
            int intTemp;
            float value;
            float powCoeff = pow(10.0f, (float) m_depthDecimalNum);
    
            auto pxPtr = reinterpret_cast<float*>(depthImage.getRawImage());
            for(int i=0; i<m_height*m_width; i++){
                value = m_depthFrame_Buffer[i];
    
                intTemp = (int) (value * powCoeff);
                value = ((float) intTemp) / powCoeff;
    
                if (value < nearPlane) { value = nearPlane; }
                if (value > farPlane) { value = farPlane; }
    
                *pxPtr = value;
                pxPtr++;
            }
        }
    
        timeStamp->update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());
    
        return true;
    }
    bool getExtrinsicParam(yarp::sig::Matrix& extrinsic) const override
    {
        extrinsic.resize(4, 4);
        extrinsic.zero();
        extrinsic[1][1] = extrinsic[2][2] = extrinsic[3][3] = extrinsic[4][4] = 1;
        return true;
    }
    bool getImages(FlexImage& colorFrame, depthImageType& depthFrame, Stamp* colorStamp, Stamp* depthStamp) const override
    {
        return getDepthImage(depthFrame, depthStamp) && getRgbImage(colorFrame, colorStamp);
    }
    
    IRGBDSensor::RGBDSensor_status getSensorStatus() override
    {
        return m_depthCameraSensorPtr->IsActive() ? RGBD_SENSOR_OK_IN_USE : RGBD_SENSOR_NOT_READY;
    }
    std::string getLastErrorMsg(Stamp* timeStamp) override
    {
        if(timeStamp)
        {
            timeStamp->update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());
    
        }
        return m_error;
    }



    // IDepthCameraData

    void setDepthCameraData(::gzyarp::DepthCameraData* dataPtr) override
    {
        m_sensorData = dataPtr;
    }

private:
    ::gzyarp::DepthCameraData* m_sensorData;
    int counter;
    bool m_vertical_flip;
    bool m_horizontal_flip;
    static double start_time;
    bool m_display_time_box;
    bool m_display_timestamp;

};
