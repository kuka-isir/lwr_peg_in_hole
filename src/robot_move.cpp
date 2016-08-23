#include <lwr_peg_in_hole/robot_move.hpp>

RobotMove::RobotMove(bool sim) : 
  spinner_(1), controller_ac("/joint_trajectory_controller/follow_joint_trajectory"), ptp_ac("ptp"), lin_rel_ac("lin_rel"), emergency_stopped_(false)
{
  sim_ = sim;
  
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
  
  // Wait for krl action servers to be running
  if (sim_){
    lin_rel_ac.waitForServer();
    ptp_ac.waitForServer();
  }
  else
    controller_ac.waitForServer();
  
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
  emerg_stopped_sub_ = nh_.subscribe("/robot_emergency_stopped", 1, &RobotMove::emergStoppedCallback,this);
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
  
  if (emergency_stopped_) {
    while (emergency_stopped_) {
      ROS_WARN("Waiting for emergency stop to be removed");
      if (!ros::ok()) 
        return false;
      ros::Duration(0.3).sleep();
    }
  }
  int num_pts = mg_plan.trajectory_.joint_trajectory.points.size();
  ROS_INFO("Executing joint trajectory with %d knots and duration %f", num_pts, 
      mg_plan.trajectory_.joint_trajectory.points[num_pts-1].time_from_start.toSec());
  
  // Copy trajectory
  control_msgs::FollowJointTrajectoryGoal traj_goal;
  traj_goal.trajectory = mg_plan.trajectory_.joint_trajectory;

  // Ask to execute now
  traj_goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.15); 

  // Specify path and goal tolerance
  //traj_goal.path_tolerance
  
  // Send goal and wait for a result
  controller_ac.sendGoal(traj_goal);
  while (ros::ok()) {
    if (controller_ac.waitForResult(ros::Duration(1.0/30.0))) 
      break;
    if (emergency_stopped_) {
      ROS_WARN("Stopping trajectory!");
      stopJointTrajectory();
      ros::Duration(0.5).sleep();
      return false;
    }
  }
  actionlib::SimpleClientGoalState end_state = controller_ac.getState();
  return end_state == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void RobotMove::stopJointTrajectory()
{
  ROS_INFO("Stopping joint trajectory");
  controller_ac.cancelGoal();
}

bool RobotMove::moveToJointPosition(const std::vector<double> joint_vals)
{
  if (sim_){
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
  }else{
    std::vector<unsigned char> mask(joint_vals.size(), 1);
    krl_msgs::PTPGoal ptp_goal;
    std::vector<float> goal_vals;
    for(int i; i<joint_vals.size();i++)
      goal_vals.push_back(joint_vals[i]);
    ptp_goal.ptp_goal_rad = goal_vals;
    ptp_goal.ptp_mask = mask;
    ptp_goal.vel_percent = 10;
    ptp_ac.sendGoal(ptp_goal);
    bool finished_before_timeout = ptp_ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ptp_ac.getState();
      ROS_INFO("PTP action finished: %s",state.toString().c_str());
      return true;
    }
    else{
      ROS_INFO("PTP action did not finish before the time out.");
      return false;
    }
  }
    
}

bool RobotMove::moveToCartesianPose(const geometry_msgs::Pose pose)
{
  
//   getPlanningScene(planning_scene_msg_, full_planning_scene_);
//   group_->getCurrentState()->update(true);
  
  if(sim_){
    // Compute ik
    ROS_INFO("Computing IK");
    sensor_msgs::JointState joints_ik;
    if (!compute_ik(pose, joints_ik)){
      ROS_ERROR("IK fail...");
      return false;
    }
    ROS_INFO("IK success!");

    // Set joint target
    group_->setJointValueTarget(joints_ik);

    // Plan trajectory
    ROS_INFO("Planning movement");
    if (!group_->plan(next_plan_)){
      ROS_ERROR("Plannning fail ...");
      return false;
    }
    ROS_INFO("Planning success !");
    
    // Execute trajectory
    ROS_INFO("Executing movement");
    if (executeJointTrajectory(next_plan_)){
      ROS_INFO("Execution success !");
      return true;
    }
    else{
      ROS_ERROR("Executing fail ...");
      return false;
    }
  }
  else{
    krl_msgs::LIN_RELGoal lin_goal;
    geometry_msgs::Vector3 xyz, xyz_mask, abc, abc_mask;
    xyz.x = pose.position.x;
    xyz.y = pose.position.y;
    xyz.z = pose.position.z;
    lin_goal.XYZ = xyz;
    xyz_mask.x = 1.0;
    xyz_mask.y = 1.0;
    xyz_mask.z = 1.0;
    lin_goal.XYZ_mask = xyz_mask;
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    abc.x = roll;
    abc.y = pitch;
    abc.z = yaw;
    lin_goal.ABC = abc;
    abc_mask.x = 1.0;
    abc_mask.y = 1.0;
    abc_mask.z = 1.0;
    lin_goal.ABC_mask = abc_mask;
    
    lin_rel_ac.sendGoal(lin_goal);
    bool finished_before_timeout = lin_rel_ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = lin_rel_ac.getState();
      ROS_INFO("LIN_REL action finished: %s",state.toString().c_str());
      return true;
    }
    else{
      ROS_INFO("LIN_REL action did not finish before the time out.");
      return false;
    }
  }
  
}

void RobotMove::emergStoppedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  emergency_stopped_ = msg->data;
}

bool RobotMove::moveToStart()
{
  if(sim_){
    group_->setNamedTarget("start");
    
    // Plan trajectory
    if (!group_->plan(next_plan_)){
      ROS_INFO("Home position motion planning failed");
      return false;
    }
    ROS_INFO("Home position motion planning successful");

    // Execute trajectory
    if (executeJointTrajectory(next_plan_)) {
      ROS_INFO("Home position joint trajectory execution successful");
      return true;
    }
    else {
      ROS_ERROR("Home position joint trajectory execution failed");
      return false;
    }
  }
  else{
    std::vector<double> start;
    start.push_back(1.6002);
    start.push_back(-0.0471);
    start.push_back(-0.3334);
    start.push_back(-1.6943);
    start.push_back(0.);
    start.push_back(1.3884);
    start.push_back(0.);
    return moveToJointPosition(start);
  }
}

bool RobotMove::moveToRandomTarget()
{
  group_->setRandomTarget();
  
  // Plan trajectory
  if (!group_->plan(next_plan_)){
    ROS_INFO("Motion planning to random target failed");
    return false;
  }
  ROS_INFO("Motion planning to random target successful");

  // Execute trajectory
  if (executeJointTrajectory(next_plan_)) {
    ROS_INFO("Joint trajectory execution to random target successful");
    return true;
  }
  else {
    ROS_ERROR("Joint trajectory execution to random target failed");
    return false;
  }
}

moveit_msgs::CollisionObjectPtr RobotMove::getCollisionObject(std::string object_name)
{
  // update the planning scene to get the robot's state
  getPlanningScene(planning_scene_msg_, full_planning_scene_);

  for(int i=0;i<planning_scene_msg_.world.collision_objects.size();i++){
    if(planning_scene_msg_.world.collision_objects[i].id == object_name){ 
      return moveit_msgs::CollisionObjectPtr(new moveit_msgs::CollisionObject(planning_scene_msg_.world.collision_objects[i]));
    }
  }
  ROS_ERROR_STREAM("Failed to find object "<< object_name<< " in the scene !!!");
  return moveit_msgs::CollisionObjectPtr();
}

bool RobotMove::moveAboveObjectHole(const std::string obj_name, const int hole_nb)
{
  ROS_INFO_STREAM("Moving above hole "<<hole_nb<<" of object "<<obj_name);
  moveit_msgs::CollisionObjectPtr coll_obj = getCollisionObject(obj_name);
  if (!coll_obj)
    return false;
  
  if (hole_nb >= holes_location_.size()){
    ROS_ERROR_STREAM("Ask to move to hole #"<<hole_nb<<" but there is only "<<holes_location_.size()<<" holes loaded");
    return false;
  }

  geometry_msgs::PoseStamped obj_pose;
  geometry_msgs::Pose target_pose;
  obj_pose.header = coll_obj->header;
  obj_pose.pose = coll_obj->mesh_poses[0];
  tf_->transformPose(base_frame_, obj_pose, obj_pose);
  
  tf::Transform object_transform;
  object_transform.setOrigin(tf::Vector3(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z));
  object_transform.setRotation(tf::Quaternion(obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z, obj_pose.pose.orientation.w));
  
  tf::Transform hole_transform;
  hole_transform.setOrigin(tf::Vector3(holes_location_[hole_nb][0], holes_location_[hole_nb][1], holes_location_[hole_nb][2]));
  tf::Quaternion rotation_to_hole;
  rotation_to_hole.setRPY(holes_location_[hole_nb][3],holes_location_[hole_nb][4],holes_location_[hole_nb][5]);
  hole_transform.setRotation(rotation_to_hole);
  
  tf::Transform up_transform;
  up_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.32));
  tf::Quaternion rotation;
  rotation.setRPY(0,0,0);
  up_transform.setRotation(rotation);
  
  tf::Transform pi_rotation_transform;
  pi_rotation_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion pi_rotation;
  pi_rotation.setRPY(M_PI,0,0);
  pi_rotation_transform.setRotation(pi_rotation);
  
  object_transform *= hole_transform;
  object_transform *= up_transform;
  object_transform *= pi_rotation_transform;
  
  target_pose.position.x = object_transform.getOrigin().getX();
  target_pose.position.y = object_transform.getOrigin().getY();
  target_pose.position.z = object_transform.getOrigin().getZ();
  target_pose.orientation.x = object_transform.getRotation().getX();
  target_pose.orientation.y = object_transform.getRotation().getY();
  target_pose.orientation.z = object_transform.getRotation().getZ();
  target_pose.orientation.w = object_transform.getRotation().getW();
  
  return this->moveToCartesianPose(target_pose);
}

bool RobotMove::loadHolesLocation(const std::string obj_name)
{
  ROS_INFO_STREAM("Loading relative holes locations for "<<obj_name);
  YAML::Node holes_config = YAML::LoadFile(ros::package::getPath("lwr_peg_in_hole")+"/holes/"+obj_name+".yaml");
  
  if (!holes_config["holes"]) {
    ROS_ERROR("Couldn't read holes location");
    return false;
  }
  YAML::Node poses = holes_config["holes"];  
  
  holes_location_.clear();
  for(unsigned int i =0; i<poses.size(); i++){
    std::vector<float> pose = poses[i].as<std::vector<float> >();
    holes_location_.push_back(pose);
  }
  
  return true;
}
