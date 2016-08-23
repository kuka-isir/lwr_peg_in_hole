#ifndef ROBOT_MOVE_HPP
#define ROBOT_MOVE_HPP

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
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
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <krl_msgs/LINAction.h>
#include <krl_msgs/PTPAction.h>
#include <std_msgs/Bool.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

#define M_PI 3.14159265358979323846  /* pi */

typedef move_group_interface::MoveGroup::Plan MoveGroupPlan;

class RobotMove
{
public:
  RobotMove(bool sim = false);
  
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
  
  // Emergency callback
  void emergStoppedCallback(const std_msgs::Bool::ConstPtr& msg);
  
    // The robot tries to go to its home position
  bool moveToStart();
  
  // The robot tries to go to a random target
  bool moveToRandomTarget();
  
  // Look for the object name in the scene and return its collision object
  moveit_msgs::CollisionObjectPtr getCollisionObject(std::string object_name);
  
  // Read yaml file with holes position and save them
  bool loadHolesLocation(const std::string obj_name);
  
  // Move above a specific hole of the specified object
  bool moveAboveObjectHole(const std::string obj_name, const int hole_nb);
  
  
  
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner_;
  boost::shared_ptr<tf::TransformListener> tf_;
  boost::scoped_ptr<move_group_interface::MoveGroup> group_;
  boost::scoped_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

  ros::ServiceClient ik_service_client_, fk_service_client_;
  moveit_msgs::GetPositionIK::Request ik_srv_req_;
  moveit_msgs::GetPositionIK::Response ik_srv_resp_;
  moveit_msgs::GetPositionFK::Request fk_srv_req_;
  moveit_msgs::GetPositionFK::Response fk_srv_resp_;
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> controller_ac;
  actionlib::SimpleActionClient<krl_msgs::PTPAction> ptp_ac;
  actionlib::SimpleActionClient<krl_msgs::LINAction> lin_ac;
  
  ros::Publisher attached_object_publisher_, planning_scene_diff_publisher_;
  ros::Subscriber emerg_stopped_sub_;
  
  moveit_msgs::PlanningScene planning_scene_msg_;
  planning_scene::PlanningScenePtr full_planning_scene_;
  
  std::string base_frame_, ee_frame_, group_name_;
  MoveGroupPlan next_plan_;
  bool sim_, emergency_stopped_;
  
  std::vector<std::vector<float> > holes_location_;
  
};

#endif