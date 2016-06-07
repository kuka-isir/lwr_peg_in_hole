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

#ifndef PEG_IN_HOLE_HPP
#define PEG_IN_HOLE_HPP

#include <ros/ros.h>
#include <ros/time.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/GetCartesianPath.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/attached_body.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <shape_msgs/Mesh.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>

#include <boost/lexical_cast.hpp>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

# define M_PI 3.14159265358979323846  /* pi */

typedef move_group_interface::MoveGroup::Plan MoveGroupPlan;

class PegInHole
{
public:
  
  //*** Class functions ***//
  
  // Constructor.
  PegInHole();
  
  // Update local planning scene variables
  void getPlanningScene(moveit_msgs::PlanningScene& planning_scene, planning_scene::PlanningScenePtr& full_planning_scene);
  
  // Compute FK
  bool compute_fk(const sensor_msgs::JointState joints, geometry_msgs::Pose &pose);
  
  // Compute IK
  bool compute_ik(const geometry_msgs::Pose pose, sensor_msgs::JointState &joints);
  
  // Get current cartesian pose
  bool getCurrentCartesianPose(geometry_msgs::Pose &pose, std::string target_frame = "");
  
  // Get current joint state
  bool getCurrentJointPosition(std::vector<double> &joints);
  
  // Execute a joint trajectory
  bool executeJointTrajectory(const MoveGroupPlan mg_plan);
  
  // Stop current joint trajectory
  void stopJointTrajectory();
  
  // The robot tries to go to the passed joint values
  bool moveToJointPosition(const std::vector<double> target_joints);
  
  // The robot tries to go to the (x,y,z) position
  bool moveToCartesianPose(const geometry_msgs::Pose target_pose);
  
  // The robot tries to go to its home position
  bool moveToStart();
  
  // The robot tries to go to a random target
  bool moveToRandomTarget();
  
  // From current pose, move arm vertically to target z
  bool verticalMove(double target_z);
  
  // From current pose, move arm vertically to target z
  bool verticalMoveBis(double target_z);
  
  // Look for the object name in the scene and return its collision object
  moveit_msgs::CollisionObjectPtr getCollisionObject(std::string object_name);

  // attach the collision model to the robot
  bool attachObject(std::string object_name);  

  // detach the collision model from the robot
  bool detachObject();
  
  // Remove all objects of the world and also the ones attached to the robot
  void cleanObjects();
  
  // Read yaml file with holes position and save them
  bool loadHolesLocation(const std::string obj_name);
  
  // Move above a specific hole of the specified object
  bool moveAboveObjectHole(const std::string obj_name, const int hole_nb);

  //*** Class variables ***//
  
  ros::AsyncSpinner spinner_;
  
  boost::shared_ptr<tf::TransformListener> tf_;
  boost::scoped_ptr<move_group_interface::MoveGroup> group_;
  boost::scoped_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

  ros::ServiceClient ik_service_client_, fk_service_client_, cartesian_path_service_client_;
  moveit_msgs::GetPositionIK::Request ik_srv_req_;
  moveit_msgs::GetPositionIK::Response ik_srv_resp_;
  moveit_msgs::GetPositionFK::Request fk_srv_req_;
  moveit_msgs::GetPositionFK::Response fk_srv_resp_;
  moveit_msgs::GetCartesianPath::Request cart_path_srv_req_;
  moveit_msgs::GetCartesianPath::Response cart_path_srv_resp_;
  
  ros::Publisher attached_object_publisher_, planning_scene_diff_publisher_;
  
  moveit_msgs::PlanningScene planning_scene_msg_;
  planning_scene::PlanningScenePtr full_planning_scene_;
  
  std::string base_frame_, ee_frame_, group_name_;
  double gripping_offset_, dz_offset_;
  MoveGroupPlan next_plan_;
  
  std::vector<std::vector<float> > holes_location_;
};

#endif