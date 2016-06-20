//| This file is a part of the sferes2 framework.
//| Copyright 2016, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jimmy Da Silva, jimmy.dasilva@isir.upmc.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software. You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifndef UPDATE_SCENE_SERVICE_HPP
#define UPDATE_SCENE_SERVICE_HPP

#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

#include <lwr_peg_in_hole/UpdateSceneService.h>

#define M_PI 3.14159265358979323846  /* pi */

class UpdateSceneService
{
  public:
    // Constructor.
    UpdateSceneService(){
      ros::WallDuration sleep_t(0.5);
      planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
      while(planning_scene_diff_publisher_.getNumSubscribers() < 1)
      {
        sleep_t.sleep();
      }
      service_server_ = nh_.advertiseService("update_scene", &UpdateSceneService::updateObjectInScene,this);
      
      ROS_INFO("Update_scene service ready !");
    };

    bool updateObjectInScene(lwr_peg_in_hole::UpdateSceneService::Request  &req, lwr_peg_in_hole::UpdateSceneService::Response &res){
      
      int tag_id = req.tag_id;
      std::string object_name = req.object_name, tag_name = "/ar_marker_"+ std::to_string(tag_id);
      
      ROS_INFO("Calling update in scene for object %s with tag %d", object_name.c_str(), tag_id);
      
      // Check if mesh exists
      std::string path_to_mesh = ros::package::getPath("lwr_peg_in_hole")+"/meshes/"+object_name+"_tag.dae";
      std::ifstream file(path_to_mesh.c_str());

      if (!file){
        ROS_ERROR_STREAM("Could not find object called "<<object_name);
        return false;
      }
      
      // Check pose of tag in mesh
      std::string path_to_config = ros::package::getPath("lwr_peg_in_hole")+"/holes/"+object_name+".yaml";
      std::ifstream config(path_to_config.c_str());
      std::vector<float> tag_pose;
      if (!config){
        ROS_WARN_STREAM("Could not find pose of tag. Will assume tag is at the origin");
        tag_pose.resize(7);
        tag_pose[6] = 1.0;
      }
      else{
        YAML::Node tag_config = YAML::LoadFile(ros::package::getPath("lwr_peg_in_hole")+"/holes/"+object_name+".yaml");
        if (!tag_config["tag_pose"]) {
          ROS_ERROR("Couldn't read tag pose in config file");
          return false;
        }
        tag_pose = tag_config["tag_pose"].as<std::vector<float> >();
      }
      tf::Transform tag_transform;
      tag_transform.setOrigin(tf::Vector3(tag_pose[0], tag_pose[1], tag_pose[2]));
      tf::Quaternion rotation_to_tag;
      rotation_to_tag.setRPY(tag_pose[3],tag_pose[4],tag_pose[5]);
      tag_transform.setRotation(rotation_to_tag);
      
      // Define the attached object message //
      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = "world";
      collision_object.id = object_name;

      // Define the mesh //
      double size = 0.0254; 
      Eigen::Vector3d scale;
      scale[0] = size;
      scale[1] = size;
      scale[2] = size;
      shapes::Mesh* m = shapes::createMeshFromResource("package://lwr_peg_in_hole/meshes/"+object_name+"_tag.dae", scale);

      shape_msgs::Mesh co_mesh;
      shapes::ShapeMsg co_mesh_msg;
      shapes::constructMsgFromShape(m,co_mesh_msg);
      co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

      // Define the object's position //
      geometry_msgs::Pose mesh_pose;
      tf::StampedTransform tform;
      try{
        tf_.waitForTransform("/world",tag_name, ros::Time(0), ros::Duration(1.0));
        tf_.lookupTransform("/world",tag_name, ros::Time(0), tform);
      }catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return false;
      }
      
      // Move transform according to the tag's pose
      tform *= tag_transform;
      
      mesh_pose.position.x = tform.getOrigin().x();
      mesh_pose.position.y = tform.getOrigin().y();
      mesh_pose.position.z = tform.getOrigin().z();
      mesh_pose.orientation.x = tform.getRotation().x();
      mesh_pose.orientation.y = tform.getRotation().y();
      mesh_pose.orientation.z = tform.getRotation().z();
      mesh_pose.orientation.w = tform.getRotation().w();
      
      // Attach object operation /y
      collision_object.meshes.push_back(co_mesh);
      collision_object.mesh_poses.push_back(mesh_pose);
      collision_object.operation = collision_object.ADD;

      // Put the object in the environment //
      planning_scene_msg_.world.collision_objects.push_back(collision_object);
      planning_scene_msg_.is_diff = true;
      planning_scene_diff_publisher_.publish(planning_scene_msg_);
      
      return true;
    };
    
    ros::NodeHandle nh_;
    ros::ServiceServer service_server_;
    ros::Publisher planning_scene_diff_publisher_;
    moveit_msgs::PlanningScene planning_scene_msg_;
    tf::TransformListener tf_;
    
};


#endif