#pragma once

#include <gz/transport/Node.hh>

#include <yarp/dev/ISimulatedWorld.h>
#include <yarp/dev/DeviceDriver.h>

namespace yarp::dev::gzyarp
{
class IWorldSharedInterface
{
public:
    virtual void setNode(gz::transport::Node* dataPtr) = 0;

    virtual ~IWorldSharedInterface() {};
};

class WorldProxy :
        public yarp::dev::DeviceDriver,
        public yarp::dev::ISimulatedWorld,
        public IWorldSharedInterface
{
private:
    gz::transport::Node*      m_node = nullptr;
    std::vector <std::string> m_hobj;
    const int                 m_timeout=1000; //ms

public:
    WorldProxy();
    ~WorldProxy();

    // Device Driver interface
    bool open(yarp::os::Searchable &config) override;
    bool close() override;

    // IWorldSharedInterface interface
    void setNode(gz::transport::Node* dataPtr) override;

    /* ISimulatedWorld methods */
    yarp::dev::ReturnValue makeBox (std::string id, double width, double height, double thickness, yarp::sig::Pose6D pose, yarp::sig::ColorRGB color, std::string frame_name, bool gravity_enable, bool collision_enable)  override;
    yarp::dev::ReturnValue makeSphere (std::string id, double radius, yarp::sig::Pose6D pose, yarp::sig::ColorRGB color, std::string frame_name, bool gravity_enable, bool collision_enable)  override;
    yarp::dev::ReturnValue makeCylinder (std::string id, double radius, double length, yarp::sig::Pose6D pose, yarp::sig::ColorRGB color, std::string frame_name, bool gravity_enable, bool collision_enable)  override;
    yarp::dev::ReturnValue makeFrame (std::string id, double size, yarp::sig::Pose6D pose, yarp::sig::ColorRGB color, std::string frame_name, bool gravity_enable, bool collision_enable)  override;
    yarp::dev::ReturnValue changeColor (std::string id, yarp::sig::ColorRGB color)  override;
    yarp::dev::ReturnValue setPose(std::string id, yarp::sig::Pose6D pose, std::string frame_name)  override;
    yarp::dev::ReturnValue enableGravity (std::string id, bool enable)  override;
    yarp::dev::ReturnValue enableCollision (std::string id, bool enable)  override;
    yarp::dev::ReturnValue getPose(std::string id, yarp::sig::Pose6D& pose,  std::string frame_name)  override;
    yarp::dev::ReturnValue makeModel(std::string id,  std::string filename, yarp::sig::Pose6D pose,  std::string frame_name, bool gravity_enable, bool collision_enable)  override;
    yarp::dev::ReturnValue deleteObject(std::string id)  override;
    yarp::dev::ReturnValue deleteAll()  override;
    yarp::dev::ReturnValue getList(std::vector<std::string>& names)  override;
    yarp::dev::ReturnValue attach(std::string id, std::string link_name)  override;
    yarp::dev::ReturnValue detach(std::string id)  override;
    yarp::dev::ReturnValue rename(std::string old_name, std::string new_name) override;
};

}
