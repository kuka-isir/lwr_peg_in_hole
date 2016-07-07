#ifndef OBJECT_SPAWN_HPP
#define OBJECT_SPAWN_HPP

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

#define M_PI 3.14159265358979323846  /* pi */

class ObjectSpawn
{
public:
  ObjectSpawn();
  
  bool updateObjectInScene(std::string& object_name, int& tag_id);
  
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner_;
  ros::Publisher planning_scene_diff_publisher_;
  tf::TransformListener tf_;

};

#endif