#include "worldproxy.h"

#include <math.h>

#include <string>
#include <iostream>
#include <ostream>

#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory_v.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/empty.pb.h>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Matrix4.hh>

using namespace std;
using namespace yarp::dev;
using namespace yarp::dev::gzyarp;

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

void replace(string &str, string key, double value)
{
    ostringstream tmp;
    tmp << value;

    size_t pos = 0;
    while ((pos = str.find(key, pos)) != string::npos) {
         str.replace(pos, key.length(), tmp.str());
         pos += tmp.str().length();
    }
}

void replace(string &str, string key, string value)
{
    size_t pos = str.find(key);
    if (pos != std::string::npos) {
        str.replace(pos, key.length(), value);
    }
}

bool WorldProxy::open(yarp::os::Searchable &config)
{
    return true;
}

bool WorldProxy::close()
{
    return true;
}    

WorldProxy::WorldProxy()
{}

WorldProxy::~WorldProxy()
{}

void WorldProxy::setNode(gz::transport::Node* dataPtr)
{
    m_node = dataPtr;
}
    
ReturnValue WorldProxy::makeBox (std::string id, double width, double height, double thickness, yarp::sig::Pose6D pose, yarp::sig::ColorRGB color, std::string frame_name, bool gravity_enable, bool collision_enable)
{
    std::string service = "/world/default/create";
    gz::msgs::EntityFactory req;

    auto it = std::find (m_hobj.begin(), m_hobj.end(), id);
    if(it != m_hobj.end()) {
       yError() << "Object: " << id << " already exists, skipping";
       return ReturnValue::return_code::return_value_error_method_failed;
    }
    
    string boxSDF_String=string(
    "<?xml version='1.0'?>\
    <sdf version ='1.4'>\
        <model name ='MODELNAME'>\
      <poseRELATIVETO>POSEX POSEY POSEZ  ROLL PITCH YAW</pose>\
      <link name ='link'>\
      <pose>0 0 0 0 0 0</pose>\
      <collision name ='collision'>\
        <geometry>\
          <box><size>WIDTH HEIGHT THICKNESS</size></box>\
        </geometry>\
      </collision>\
      <visual name='visual'>\
        <geometry>\
          <box><size>WIDTH HEIGHT THICKNESS</size></box>\
        </geometry>\
        <material>\
                   <ambient>RED GREEN BLUE 1</ambient>\
                   <diffuse>RED GREEN BLUE 1</diffuse>\
        </material>\
       </visual>\
       <gravity>GRAVITY</gravity>\
       </link>\
      </model>\
    </sdf>");

    if (frame_name=="") {replace(boxSDF_String, "RELATIVETO", "");}
    else                {replace(boxSDF_String, "RELATIVETO", std::string(" relative_to=\"")+ frame_name +"\" ");}
    
    replace(boxSDF_String, "MODELNAME", id);
    replace(boxSDF_String, "POSEX", pose.x);
    replace(boxSDF_String, "POSEY", pose.y);
    replace(boxSDF_String, "POSEZ", pose.z);
    replace(boxSDF_String, "ROLL",  pose.roll);
    replace(boxSDF_String, "PITCH", pose.pitch);
    replace(boxSDF_String, "YAW",   pose.yaw);

    replace(boxSDF_String, "WIDTH", width);
    replace(boxSDF_String, "HEIGHT", height);
    replace(boxSDF_String, "THICKNESS", thickness);

    replace(boxSDF_String, "RED", color.r/255.0);
    replace(boxSDF_String, "GREEN", color.g/255.0);
    replace(boxSDF_String, "BLUE", color.b/255.0);
    
    if (gravity_enable) {replace (boxSDF_String, "GRAVITY", 1);}
    else {replace (boxSDF_String, "GRAVITY", 0);}
  
    // Set the sdf
    req.set_sdf(boxSDF_String);

    // Reply
    gz::msgs::Boolean rep;
    bool result;

    bool success = m_node->Request(service, req, m_timeout, rep, result);

    if (success && result && rep.data())
    {
       yInfo() << "Object creatated:" << id;
    }
    else
    {
       yError() << "Unable to create object:" << id;
       return ReturnValue::return_code::return_value_error_method_failed;
    }
    
    //update the list of handled objects
    m_hobj.push_back(id);
    
    if (collision_enable == false) { yError()<< "Gravity Not yet implemented"; }
    return ReturnValue_ok;
}

ReturnValue WorldProxy::makeSphere (std::string id, double radius, yarp::sig::Pose6D pose, yarp::sig::ColorRGB color, std::string frame_name, bool gravity_enable, bool collision_enable)
{
    yError() << "Not yet implemented";
    string sphereSDF_string=string(
      "<?xml version='1.0'?>\
       <sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>RADIUS</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>RADIUS</radius></sphere>\
                </geometry>\
                  <material>\
                  <ambient>RED GREEN BLUE 1</ambient>\
                  <diffuse>RED GREEN BLUE 1</diffuse>\
                  </material>\
              </visual>\
              <gravity>GRAVITY</gravity>\
            </link>\
          </model>\
        </sdf>");
   return ReturnValue_ok;
}

ReturnValue WorldProxy::makeCylinder (std::string id, double radius, double length, yarp::sig::Pose6D pose, yarp::sig::ColorRGB color, std::string frame_name, bool gravity_enable, bool collision_enable)
{
    yError() << "Not yet implemented";
    string cylSDF_String=string(
    "<?xml version='1.0'?>\
    <sdf version ='1.4'>\
        <model name ='cylinder'>\
      <pose>POSEX POSEY POSEZ  ROLL PITCH YAW</pose>\
         <link name ='link'>\
      <pose>0 0 0 0 0 0</pose>\
      <collision name ='collision'>\
        <geometry>\
          <cylinder><radius>RADIUS</radius><length>LENGTH</length></cylinder>\
        </geometry>\
      </collision>\
      <visual name='visual'>\
        <geometry>\
          <cylinder><radius>RADIUS</radius><length>LENGTH</length></cylinder>\
      </geometry>\
      <material>\
                   <ambient>RED GREEN BLUE 1</ambient>\
                   <diffuse>RED GREEN BLUE 1</diffuse>\
          </material>\
    </visual>\
    <gravity>GRAVITY</gravity>\
    </link>\
      </model>\
   </sdf>");
   return ReturnValue_ok;
}

ReturnValue WorldProxy::makeFrame (std::string id, double size, yarp::sig::Pose6D pose, yarp::sig::ColorRGB color, std::string frame_name, bool gravity_enable, bool collision_enable)
{
    yError() << "Not yet implemented";
    string frameSDF_string=string(
      "<?xml version='1.0'?>\
       <sdf version ='1.4'>\
          <model name ='frame'>\
            <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
            \
            <link name ='link'>\
             \
              <visual name ='visual_z'>\
                <pose>0 0 HLENGHT 0 0 0</pose>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>0 0 1 1</ambient>\
                  <diffuse>0 0 1 1</diffuse>\
                  </material>\
              </visual>\
              \
              <visual name ='visual_y'>\
              <pose>0 HLENGHT 0 1.5707 0 0</pose>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>0 1 0 1</ambient>\
                  <diffuse>0 1 0 1</diffuse>\
                  </material>\
              </visual>\
              \
              <visual name ='visual_x'>\
              <pose>HLENGHT 0 0 0 1.5707 0</pose>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>1 0 0 1</ambient>\
                  <diffuse>1 0 0 1</diffuse>\
                  </material>\
              </visual>\
            \
             <visual name='visual_ball'>\
             <pose>0 0 0 0 0 0</pose>\
             <geometry>\
               <sphere><radius>BRADIUS</radius></sphere>\
              </geometry>\
             <material>\
               <ambient>RED GREEN BLUE 1</ambient>\
                <diffuse>RED GREEN BLUE 1</diffuse>\
               </material>\
            </visual>\
            <gravity>GRAVITY</gravity>\
            <inertial>\
                <mass>1e-21</mass>\
            </inertial>\
            \
            </link>\
          </model>\
        </sdf>");
   return ReturnValue_ok;
}

ReturnValue WorldProxy::changeColor (std::string id, yarp::sig::ColorRGB color)
{
   return ReturnValue_ok;
}

ReturnValue WorldProxy::setPose(std::string id, yarp::sig::Pose6D pose, std::string frame_name)
{
   std::string service = "/world/default/set_pose";
        
   gz::msgs::Pose req;
   req.set_name(id);

   gz::math::Quaterniond q (pose.pitch, pose.roll, pose.yaw);
   req.mutable_position()->set_x(pose.x);
   req.mutable_position()->set_y(pose.y);
   req.mutable_position()->set_z(pose.z);

   req.mutable_orientation()->set_x(q.X());
   req.mutable_orientation()->set_y(q.Y());
   req.mutable_orientation()->set_z(q.Z());
   req.mutable_orientation()->set_w(q.W());

   // Reply
   gz::msgs::Boolean rep;
   bool result;

   bool success = m_node->Request(service, req, m_timeout, rep, result);

   if (success && result && rep.data())
   {
      yInfo() << "Moved object:" << id;
   }
   else
   {
      yError() << "Unable to move object:" << id;
      return ReturnValue::return_code::return_value_error_method_failed;
   }
    
   return ReturnValue_ok;
}

ReturnValue WorldProxy::enableGravity (std::string id, bool enable)
{
   return ReturnValue_ok;
}

ReturnValue WorldProxy::enableCollision (std::string id, bool enable)
{
   return ReturnValue_ok;
}

gz::msgs::Pose computeRelative (gz::msgs::Pose pose_a, gz::msgs::Pose pose_b)
{
    gz::math::Vector3d pos_a(pose_a.position().x(), pose_a.position().y(), pose_a.position().z());
    gz::math::Quaterniond rot_a(pose_a.orientation().w(), pose_a.orientation().x(), pose_a.orientation().y(), pose_a.orientation().z()); 

    gz::math::Vector3d pos_b(pose_b.position().x(), pose_b.position().y(), pose_b.position().z());
    gz::math::Quaterniond rot_b(pose_b.orientation().w(), pose_b.orientation().x(), pose_b.orientation().y(), pose_b.orientation().z());

    gz::math::Matrix4d T_a (rot_a);
    T_a.SetTranslation(pos_a);
    gz::math::Matrix4d T_b (rot_b);
    T_b.SetTranslation(pos_b);

    gz::math::Matrix4d T_relative = T_a.Inverse() * T_b;

    gz::msgs::Pose relative_pose;
    relative_pose.mutable_position()->set_x(T_relative.Translation().X());
    relative_pose.mutable_position()->set_y(T_relative.Translation().Y());
    relative_pose.mutable_position()->set_z(T_relative.Translation().Z());
    relative_pose.mutable_orientation()->set_x(T_relative.Rotation().X());
    relative_pose.mutable_orientation()->set_y(T_relative.Rotation().Y());
    relative_pose.mutable_orientation()->set_z(T_relative.Rotation().Z());
    relative_pose.mutable_orientation()->set_w(T_relative.Rotation().W());
    return relative_pose;
}

ReturnValue WorldProxy::getPose(std::string id, yarp::sig::Pose6D& pose,  std::string frame_name)
{
    std::string service = "/world/default/scene/info";
    
    // Reply
    gz::msgs::Empty req;
    gz::msgs::Scene rep;
    bool result;

    bool success = m_node->Request(service, req, m_timeout, rep, result);

    if (!success || !result)
    {
       yError() << "Unable to prorcess entity/info request";
       return ReturnValue::return_code::return_value_error_method_failed;
    }

    for (size_t i = 0; i < rep.model_size(); ++i)
    {
        const auto &model = rep.model(i);
        if (model.name() == id)
        {
            //for (size_t j = 0; j < model.link_size(); ++j)
            //{
            //}
            const auto &p = model.pose();
            #if 0
            yDebug() << "Model: " << model.name() << "\n";
            yDebug() << "Position: "
                      << p.position().x() << ", "
                      << p.position().y() << ", "
                      << p.position().z() ;
            yDebug() << "Orientation: "
                      << p.orientation().w() << ", "
                      << p.orientation().x() << ", "
                      << p.orientation().y() << ", "
                      << p.orientation().z() ;
            #endif
            gz::math::Quaterniond q (p.orientation().x(),p.orientation().y(),p.orientation().z(),p.orientation().w());
            pose.x = p.position().x();
            pose.y = p.position().y();
            pose.z = p.position().z();
            pose.pitch = q.Pitch();
            pose.roll = q.Roll();
            pose.yaw = q.Yaw();
            
            return ReturnValue_ok;
        }
    }
    
    yError() << "Model " << id << "not found";
    return ReturnValue::return_code::return_value_error_method_failed;
}

ReturnValue WorldProxy::makeModel(std::string id,  std::string filename, yarp::sig::Pose6D pose,  std::string frame_name, bool gravity_enable, bool collision_enable)
{
    std::string service = "/world/default/create";
    gz::msgs::EntityFactory req;

    auto it = std::find (m_hobj.begin(), m_hobj.end(), id);
    if(it != m_hobj.end()) {
       yError() << "Object: " << id << " already exists, skipping";
       return ReturnValue::return_code::return_value_error_method_failed;
    }
    
    string boxSDF_String=string(
    "<?xml version='1.0'?>\
    <sdf version ='1.4'>\
        <model name ='MODELNAME'>\
          <poseRELATIVETO>POSEX POSEY POSEZ  ROLL PITCH YAW</pose>\
          <link name ='link'>\
            <pose>0 0 0 0 0 0</pose>\
            <collision name ='collision'>\
            </collision>\
            <visual name='visual'>\
              <geometry>\
                <mesh>\
                <uri>model://DAEFILE</uri>\
                <scale>SCALE SCALE SCALE</scale>\
                </mesh>\
<!--                <mesh filename=\"package://DAEFILE\" scale=\"1.0 1.0 1.0\"/>  --> \
              </geometry>\
            </visual>\
            <gravity>GRAVITY</gravity>\
          </link>\
        </model>\
    </sdf>");

    if (frame_name=="") {replace(boxSDF_String, "RELATIVETO", "");}
    else                {replace(boxSDF_String, "RELATIVETO", std::string(" relative_to=\"")+ frame_name +"\" ");}
    
    replace(boxSDF_String, "MODELNAME", id);
    replace(boxSDF_String, "POSEX", pose.x);
    replace(boxSDF_String, "POSEY", pose.y);
    replace(boxSDF_String, "POSEZ", pose.z);
    replace(boxSDF_String, "ROLL",  pose.roll);
    replace(boxSDF_String, "PITCH", pose.pitch);
    replace(boxSDF_String, "YAW",   pose.yaw);
    replace(boxSDF_String, "SCALE",   0.001);
    replace(boxSDF_String, "DAEFILE",filename);

    if (gravity_enable) {replace (boxSDF_String, "GRAVITY", 1);}
    else {replace (boxSDF_String, "GRAVITY", 0);}
  
    // Set the sdf
    // yDebug() << boxSDF_String;
    req.set_sdf(boxSDF_String);

    // Reply
    gz::msgs::Boolean rep;
    bool result;

    bool success = m_node->Request(service, req, m_timeout, rep, result);

    if (success && result && rep.data())
    {
       yInfo() << "Object creatated:" << id;
    }
    else
    {
       yError() << "Unable to create object:" << id;
       return ReturnValue::return_code::return_value_error_method_failed;
    }
    
    //update the list of handled objects
    m_hobj.push_back(id);
    
    if (collision_enable == false) { yError()<< "Gravity Not yet implemented"; }
    return ReturnValue_ok;
}

ReturnValue WorldProxy::deleteObject(std::string id)
{
    std::string service = "/world/default/remove";

    gz::msgs::Entity req;
    req.set_name(id);
    req.set_type(gz::msgs::Entity::MODEL);

    // Reply
    gz::msgs::Boolean rep;
    bool result;

    bool success = m_node->Request(service, req, m_timeout, rep, result);
    
    if (success && result && rep.data())
    {
       yInfo() << "Deleted object:" << id;
    }
    else
    {
       yError() << "Unable to delete object:" << id;
       return ReturnValue::return_code::return_value_error_method_failed;
    }

    //update the list of handled objects
    auto it = std::find (m_hobj.begin(), m_hobj.end(), id);
    if(it != m_hobj.end()) {
        m_hobj.erase(it);
    }

    return ReturnValue_ok;
}

ReturnValue WorldProxy::deleteAll()
{
   return ReturnValue_ok;
}

ReturnValue WorldProxy::getList(std::vector<std::string>& names)
{
    std::string service = "/world/default/scene/info";
    
    // Reply
    gz::msgs::Empty req;
    gz::msgs::Scene rep;
    bool result;

    bool success = m_node->Request(service, req, m_timeout, rep, result);

    if (!success || !result)
    {
       yError() << "Unable to prorcess entity/info request";
       return ReturnValue::return_code::return_value_error_method_failed;
    }

    yDebug() << "Found: " << rep.model_size() << " models";

    names.clear();
    for (size_t i = 0; i < rep.model_size(); ++i)
    {
      const auto &model = rep.model(i);
      names.push_back(model.name());
      yDebug() << "Model: " << model.name();
    }
   
    return ReturnValue_ok;
}

ReturnValue WorldProxy::attach(std::string id, std::string link_name)
{
   yError() << "Not yet implemented";
   return ReturnValue_ok;
}

ReturnValue WorldProxy::detach(std::string id)
{
   yError() << "Not yet implemented";
   return ReturnValue_ok;
}

ReturnValue WorldProxy::rename(std::string old_name, std::string new_name)
{
   yError() << "Not yet implemented";
   return ReturnValue_ok;
}

/*
///////////////////////////////////////////////////////
// The following methods still need to be implemented
///////////////////////////////////////////////////////

gazebo::physics::LinkPtr WorldProxy::HELPER_getLink(std::string full_scoped_link_name)
{
    size_t firstcolon = full_scoped_link_name.find(":");
    if (firstcolon == std::string::npos)
    {
      yError () << "Unable to parse model name: " << full_scoped_link_name;
      return gazebo::physics::LinkPtr();
    }
    std::string model_name = full_scoped_link_name.substr(0,firstcolon);
    physics::ModelPtr p_model=world->ModelByName(model_name);
    if (!p_model)
    {
      yError () << "Unable to find model: " << model_name;
      return gazebo::physics::LinkPtr();
    }

    gazebo::physics::Link_V model_links = p_model->GetLinks();
    for(int i=0; i < model_links.size(); i++ )
    {
        std::string candidate_link = model_links[i]->GetScopedName();
        if( candidate_link==full_scoped_link_name )
        {
            return model_links[i];
        }
    }
    yError () << "Unable to find link: " << full_scoped_link_name << "belonging to model: " <<model_name;
    return gazebo::physics::LinkPtr();
}

gazebo::physics::LinkPtr WorldProxy::HELPER_getLinkInModel(gazebo::physics::ModelPtr model, std::string link_name)
{
    gazebo::physics::Link_V model_links = model->GetLinks();
    for(int i=0; i < model_links.size(); i++ )
    {
        std::string candidate_link = model_links[i]->GetScopedName();
        if( HELPER_hasEnding(candidate_link,"::"+link_name) )
        {
            return model_links[i];
        }
    }
    return gazebo::physics::LinkPtr();
}

bool WorldProxy::attach(const std::string& id, const std::string& link_name)
{
    physics::ModelPtr object_model_1=world->ModelByName(id);
    if (!object_model_1)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }

    physics::JointPtr joint;
    physics::PhysicsEnginePtr physics=world->Physics();
    joint = physics->CreateJoint("revolute", object_model_1);
    if( !joint )
    {
        yError() << "Unable to create joint";
        return false;
    }

    physics::LinkPtr parent_link = object_model_1->GetLink();
    physics::LinkPtr object_link = HELPER_getLink(link_name);
    //physics::LinkPtr object_link = HELPER_getLinkInModel(object_model_2,link_name);

    if( !object_link )
    {
        yError() << "Unable to get object_link: " << link_name;
        return false;
    }
    if( !parent_link )
    {
        yError() << "Unable to get parent link: " << id;
        return false;
    }

    //YarpWorldPose parent_link_pose = parent_link->GetWorldPose();
    //object_link->SetWorldPose(parent_link_pose);

    //TODO add mutex
    joint->SetName("magnet_joint");
    joint->SetModel(object_model_1);
    joint->Load(parent_link, object_link, YarpWorldPose());
    joint->Attach(parent_link, object_link);
    joint->SetUpperLimit(0, 0);
    joint->SetLowerLimit(0, 0);
    //joint->SetParam("cfm", 0, 0);
    //yDebug() << object_model_1->GetJointCount() << object_model_2->GetJointCount();

    return true;
}

bool WorldProxy::detach(const std::string& id)
{
    physics::ModelPtr object_model=world->ModelByName(id);
    if (!object_model)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }

    yError() << "^^" << object_model->GetJointCount();
    physics::JointPtr joint = object_model->GetJoint ("magnet_joint");
    if (!joint)
    {
      yError() <<"Joint not found";
      return false;
    }

    return true;
}
*/
