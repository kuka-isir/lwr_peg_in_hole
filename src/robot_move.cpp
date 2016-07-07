#include <lwr_peg_in_hole/robot_move.hpp>

RobotMove::RobotMove() : 
  spinner_(1)
{
  // Start AsyncSpinner
  spinner_.start();
  
  ros::NodeHandle nh_param("~");
  nh_param.param<std::string>("base_frame", base_frame_ , "base_link");
  nh_param.param<std::string>("ee_frame", ee_frame_, "link_7");
  nh_param.param<std::string>("group_name", group_name_, "arm");
  
  // Initialize move group
  group_.reset(new move_group_interface::MoveGroup(group_name_));
  group_->setPlanningTime(10.0);
  group_->allowReplanning(false);
  group_->startStateMonitor(1.0);
  group_->setPlannerId("RRTConnectkConfigDefault");
  group_->setEndEffectorLink(ee_frame_);
  group_->setPoseReferenceFrame(ee_frame_);
  group_->setGoalPositionTolerance(0.01);
  group_->setGoalOrientationTolerance(0.01);

  // Configure service calls
  fk_srv_req_.header.frame_id = base_frame_;
  fk_srv_req_.fk_link_names.push_back(ee_frame_);
  ik_srv_req_.ik_request.group_name = group_name_;
  ik_srv_req_.ik_request.pose_stamped.header.frame_id = base_frame_;
  ik_srv_req_.ik_request.attempts = 100;
  ik_srv_req_.ik_request.timeout = ros::Duration(0.1);
  ik_srv_req_.ik_request.ik_link_name = ee_frame_;
  ik_srv_req_.ik_request.ik_link_names.push_back(ee_frame_);
  ik_srv_req_.ik_request.avoid_collisions = true;
  
  // Initialize planning scene monitor
  tf_.reset(new tf::TransformListener(ros::Duration(1.0)));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_));
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startStateMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  
  // Wait until the required ROS services are available
  ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
  fk_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");
  while(!ik_service_client_.exists() || !fk_service_client_.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }

  // Wait for subscribers to make sure we can publish attached/unattached objects //
  attached_object_publisher_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
  planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(attached_object_publisher_.getNumSubscribers() < 1 || planning_scene_diff_publisher_.getNumSubscribers() < 1)
  {
    ROS_INFO("Waiting for planning scene");
    sleep(1.0);
  }
  
  // Add some extra sleep to make sure the planning scene is loaded
  usleep(1000000*3);
}

void RobotMove::getPlanningScene(moveit_msgs::PlanningScene& planning_scene, planning_scene::PlanningScenePtr& full_planning_scene)
{
  planning_scene_monitor_->requestPlanningSceneState();
  full_planning_scene = planning_scene_monitor_->getPlanningScene();
  full_planning_scene->getPlanningSceneMsg(planning_scene);
}

bool RobotMove::compute_fk(const sensor_msgs::JointState joints, geometry_msgs::Pose &pose)
{
  // Update planning scene and robot state
//   getPlanningScene(planning_scene_msg_, full_planning_scene_);
  
  fk_srv_req_.header.stamp = ros::Time::now();
  fk_srv_req_.robot_state = planning_scene_msg_.robot_state;
  fk_srv_req_.robot_state.joint_state = joints;
  fk_service_client_.call(fk_srv_req_, fk_srv_resp_);
  
  if(fk_srv_resp_.error_code.val !=1)
    return false;
  
  pose = fk_srv_resp_.pose_stamped[0].pose;
  return true;
}

bool RobotMove::compute_ik(const geometry_msgs::Pose pose, sensor_msgs::JointState &joints)
{
  // Update planning scene and robot state
//   getPlanningScene(planning_scene_msg_, full_planning_scene_);
//   group_->getCurrentState()->update(true);
  
  // setup IK request
//   ik_srv_req_.ik_request.robot_state = planning_scene_msg_.robot_state;
  ik_srv_req_.ik_request.pose_stamped.header.stamp = ros::Time::now();
  ik_srv_req_.ik_request.pose_stamped.header.frame_id = base_frame_;
  ik_srv_req_.ik_request.pose_stamped.pose = pose;
  
  ik_service_client_.call(ik_srv_req_, ik_srv_resp_);
  if(ik_srv_resp_.error_code.val !=1)
    return false;

  joints = ik_srv_resp_.solution.joint_state;
  
  return true;
}

bool RobotMove::getCurrentCartesianPose(geometry_msgs::Pose &pose, std::string target_frame)
{
  // Update planning scene and robot state
  getPlanningScene(planning_scene_msg_, full_planning_scene_);
  
  // Call IK with current joint state
//   return compute_fk(planning_scene_msg_.robot_state.joint_state, pose);
  pose = group_->getCurrentPose(target_frame).pose;
  return true;
}

bool RobotMove::getCurrentJointPosition(std::vector<double> &joints)
{
  // Update planning scene and robot state
  getPlanningScene(planning_scene_msg_, full_planning_scene_);
  joints = planning_scene_msg_.robot_state.joint_state.position;
  
  return true;
}

bool RobotMove::executeJointTrajectory(const MoveGroupPlan mg_plan)
{
  return group_->execute(mg_plan);
}

void RobotMove::stopJointTrajectory()
{
  group_->stop();
}

bool RobotMove::moveToJointPosition(const std::vector<double> joint_vals)
{
//   getPlanningScene(planning_scene_msg_, full_planning_scene_);
//   group_->getCurrentState()->update(true);
  
  // Set joint target
  group_->setJointValueTarget(joint_vals);

  // Plan trajectory
  if (!group_->plan(next_plan_))
    return false;

  // Execute trajectory
  if (executeJointTrajectory(next_plan_))
    return true;
  else
    return false;
}

bool RobotMove::moveToCartesianPose(const geometry_msgs::Pose pose)
{
  
//   getPlanningScene(planning_scene_msg_, full_planning_scene_);
//   group_->getCurrentState()->update(true);
  
  // Compute ik
  sensor_msgs::JointState joints_ik;
  if (!compute_ik(pose, joints_ik))
    return false;

  // Set joint target
  group_->setJointValueTarget(joints_ik);

  // Plan trajectory
  if (!group_->plan(next_plan_))
    return false;

  // Execute trajectory
  if (executeJointTrajectory(next_plan_))
    return true;
  else
    return false;
}