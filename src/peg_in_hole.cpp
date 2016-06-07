#include <lwr_peg_in_hole/peg_in_hole.hpp>

PegInHole::PegInHole() : 
  spinner_(1)
{
  // Start AsyncSpinner
  spinner_.start();
  
  // Get params
  double max_planning_time;
  ros::NodeHandle nh, nh_param("~");
  nh_param.param<std::string>("base_frame", base_frame_ , "base_link");
  nh_param.param<std::string>("ee_frame", ee_frame_, "link_7");
  nh_param.param<std::string>("group_name", group_name_, "arm");
  nh_param.param<double>("max_planning_time", max_planning_time, 8.0);
  nh_param.param<double>("gripping_offset", gripping_offset_, 0.1);
  nh_param.param<double>("dz_offset", dz_offset_, 0.3);
  
  // Initialize move group
  group_.reset(new move_group_interface::MoveGroup(group_name_));
  group_->setPlanningTime(max_planning_time);
  group_->allowReplanning(false);
  // TODO What is this 1.0 exactly ?
  group_->startStateMonitor(1.0);
  group_->setPlannerId("RRTConnectkConfigDefault");
  group_->setEndEffectorLink(ee_frame_);
  group_->setPoseReferenceFrame(ee_frame_);
  group_->setGoalPositionTolerance(0.001);
  group_->setGoalOrientationTolerance(0.001);

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
  cart_path_srv_req_.group_name = group_name_;
  cart_path_srv_req_.header.frame_id = base_frame_;
  cart_path_srv_req_.max_step = 0.05;
  cart_path_srv_req_.jump_threshold = 0.0;
  cart_path_srv_req_.avoid_collisions = true;
  cart_path_srv_req_.link_name = ee_frame_;
  
  // Initialize planning scene monitor
  tf_.reset(new tf::TransformListener(ros::Duration(2.0)));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_));
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startStateMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  
  // Wait until the required ROS services are available
  ik_service_client_ = nh.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
  fk_service_client_ = nh.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");
  cartesian_path_service_client_ = nh.serviceClient<moveit_msgs::GetCartesianPath>(move_group::CARTESIAN_PATH_SERVICE_NAME);
  while(!ik_service_client_.exists() || !fk_service_client_.exists() || !cartesian_path_service_client_.exists() )
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }

  // Wait for subscribers to make sure we can publish attached/unattached objects //
  attached_object_publisher_ = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
  planning_scene_diff_publisher_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(attached_object_publisher_.getNumSubscribers() < 1 || planning_scene_diff_publisher_.getNumSubscribers() < 1)
  {
    ROS_INFO("Waiting for planning scene");
    sleep(1.0);
  }
  
  // Add some extra sleep to make sure the planning scene is loaded
  usleep(1000000*3);
}

void PegInHole::getPlanningScene(moveit_msgs::PlanningScene& planning_scene, planning_scene::PlanningScenePtr& full_planning_scene)
{
  planning_scene_monitor_->requestPlanningSceneState();
  full_planning_scene = planning_scene_monitor_->getPlanningScene();
  full_planning_scene->getPlanningSceneMsg(planning_scene);
}

bool PegInHole::compute_fk(const sensor_msgs::JointState joints, geometry_msgs::Pose &pose)
{
  // Update planning scene and robot state
//   getPlanningScene(planning_scene_msg_, full_planning_scene_);
  
  fk_srv_req_.header.stamp = ros::Time::now();
  fk_srv_req_.robot_state = planning_scene_msg_.robot_state;
  fk_srv_req_.robot_state.joint_state = joints;
  fk_service_client_.call(fk_srv_req_, fk_srv_resp_);
  
  if(fk_srv_resp_.error_code.val !=1){
    ROS_ERROR("FK couldn't find a solution (error code %d)", fk_srv_resp_.error_code.val);
    return false;
  }
  
  ROS_INFO("ee_frame has pose (%.2f, %.2f, %.2f)", fk_srv_resp_.pose_stamped[0].pose.position.x, fk_srv_resp_.pose_stamped[0].pose.position.y, fk_srv_resp_.pose_stamped[0].pose.position.z);
  pose = fk_srv_resp_.pose_stamped[0].pose;
  return true;
}

bool PegInHole::compute_ik(const geometry_msgs::Pose pose, sensor_msgs::JointState &joints)
{
  // Update planning scene and robot state
//   getPlanningScene(planning_scene_msg_, full_planning_scene_);
//   group_->getCurrentState()->update(true);
  
//   std::vector<double> test_joints;
//   this->getCurrentJointPosition(test_joints);
  
  // setup IK request
//   ik_srv_req_.ik_request.robot_state = planning_scene_msg_.robot_state;
  ik_srv_req_.ik_request.pose_stamped.header.stamp = ros::Time::now();
  ik_srv_req_.ik_request.pose_stamped.header.frame_id = base_frame_;
  ik_srv_req_.ik_request.pose_stamped.pose = pose;
  
  ik_service_client_.call(ik_srv_req_, ik_srv_resp_);
  if(ik_srv_resp_.error_code.val !=1){
    ROS_ERROR("IK couldn't find a solution (error code %d) for pose:\n X=(%f, %f, %f) Quat=(%f, %f, %f, %f))", 
              ik_srv_resp_.error_code.val, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    return false;
  }
  ROS_INFO("IK returned succesfully");

  joints = ik_srv_resp_.solution.joint_state;
  
//   this->IKCorrection(joints);
  
  return true;
}

bool PegInHole::getCurrentCartesianPose(geometry_msgs::Pose &pose, std::string target_frame)
{
  // Update planning scene and robot state
  getPlanningScene(planning_scene_msg_, full_planning_scene_);
  
  // Call IK with current joint state
//   return compute_fk(planning_scene_msg_.robot_state.joint_state, pose);
  pose = group_->getCurrentPose(target_frame).pose;
  return true;
}

bool PegInHole::getCurrentJointPosition(std::vector<double> &joints)
{
  // Update planning scene and robot state
  getPlanningScene(planning_scene_msg_, full_planning_scene_);
  
  joints = planning_scene_msg_.robot_state.joint_state.position;
  
  for (int i=0; i<joints.size() ;i++){
    ROS_WARN_STREAM("Joint "<<i<<" is : "<<joints[i]);
  }
  
  return true;
}

bool PegInHole::executeJointTrajectory(const MoveGroupPlan mg_plan)
{
  int num_pts = mg_plan.trajectory_.joint_trajectory.points.size();
  ROS_INFO("Executing joint trajectory with %d knots and duration %f", num_pts, 
      mg_plan.trajectory_.joint_trajectory.points[num_pts-1].time_from_start.toSec());
  
  return group_->execute(mg_plan);
}

void PegInHole::stopJointTrajectory()
{
  ROS_INFO("Stopping current joint trajectory");
  group_->stop();
}

bool PegInHole::moveToJointPosition(const std::vector<double> joint_vals)
{
//   getPlanningScene(planning_scene_msg_, full_planning_scene_);
//   group_->getCurrentState()->update(true);
  
  // Set joint target
  group_->setJointValueTarget(joint_vals);

  // Plan trajectory
  if (!group_->plan(next_plan_)){
      ROS_INFO("Motion planning to joint position failed");
    return false;
  }
  ROS_INFO("Motion planning to joint position successful");

  // Execute trajectory
  if (executeJointTrajectory(next_plan_)) {
    ROS_INFO("Trajectory execution successful");
    return true;
  }
  else {
    ROS_ERROR("Trajectory execution failed");
    return false;
  }
}

bool PegInHole::moveToCartesianPose(const geometry_msgs::Pose pose)
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
  if (!group_->plan(next_plan_)){
      ROS_INFO("Motion planning to position (%.2f, %.2f, %.2f) failed", 
      pose.position.x, pose.position.y, pose.position.z);
    return false;
  }
  ROS_INFO("Motion planning to position (%.2f, %.2f, %.2f) successful", 
      pose.position.x, pose.position.y, pose.position.z);

  // Execute trajectory
  if (executeJointTrajectory(next_plan_)) {
    ROS_INFO("Trajectory execution successful");
    return true;
  }
  else {
    ROS_ERROR("Trajectory execution failed");
    return false;
  }
}

bool PegInHole::moveToStart()
{
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

bool PegInHole::moveToRandomTarget()
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

bool PegInHole::verticalMove(double target_z)
{
  ROS_INFO("Vertical move to target z: %f", target_z);

  // Set two waypoints for the linear trajectory
  geometry_msgs::Pose pose1, pose2;
  getCurrentCartesianPose(pose1);
  pose2 = pose1;
  ROS_INFO("Calling cart path from pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
      pose1.position.x, pose1.position.y, pose1.position.z, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w);
  pose1.position.z = (pose1.position.z+target_z)/2.0;
  pose2.position.z = target_z;
  ROS_INFO("for pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
      pose2.position.x, pose2.position.y, pose2.position.z, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w);
  
  // Find linear trajectory
  moveit_msgs::RobotTrajectory lin_traj_msg;
  std::vector<geometry_msgs::Pose> waypoints;
//   waypoints.push_back(pose1);
  waypoints.push_back(pose2);

  // Call cartesian path service
  cart_path_srv_req_.header.stamp = ros::Time::now();
  cart_path_srv_req_.waypoints = waypoints;
  robot_state::robotStateToRobotStateMsg(*group_->getCurrentState(), cart_path_srv_req_.start_state);
  cartesian_path_service_client_.call(cart_path_srv_req_,cart_path_srv_resp_);
  if (cart_path_srv_resp_.error_code.val != 1) {
    ROS_ERROR("Cartesian path service returned with error code %d", cart_path_srv_resp_.error_code.val);
    return false;
  }
  if(cart_path_srv_resp_.fraction < 0.0) {
    ROS_ERROR("Failed compute a correct CartesianPath");
    return false;
  }

  // Execute plan
  MoveGroupPlan lin_traj_plan;
  lin_traj_plan.trajectory_ = cart_path_srv_resp_.solution;
  if (executeJointTrajectory(lin_traj_plan)) {
    ROS_INFO("Vertical joint trajectory execution successful");
    return true;
  }
  else {
    ROS_ERROR("Vertical joint trajectory execution failed, going to restart");
    return false;
  }
}

bool PegInHole::verticalMoveBis(double target_z)
{
  ROS_INFO("Vertical move to target z: %f", target_z);

  // Update planning scene and robot state
  getPlanningScene(planning_scene_msg_, full_planning_scene_);

  // Set two waypoints for the linear trajectory
  geometry_msgs::Pose pose = group_->getCurrentPose(ee_frame_).pose;
  pose.position.z = target_z;
  
  moveit_msgs::Constraints constraints;
  moveit_msgs::OrientationConstraint ocm;
  ocm.header.frame_id = base_frame_;
  ocm.header.stamp = ros::Time::now();
  ocm.orientation = pose.orientation;
  ocm.link_name = ee_frame_;
  ocm.absolute_x_axis_tolerance = 0.5;
  ocm.absolute_y_axis_tolerance = 0.5;
  ocm.absolute_z_axis_tolerance = 3.14;
  ocm.weight = 1.0;
  constraints.orientation_constraints.push_back(ocm);
  group_->setPathConstraints(constraints);
  
  // Compute ik
  sensor_msgs::JointState joints_ik;
  if (!compute_ik(pose, joints_ik))
    return false;
  
  // Set joint target
  group_->setJointValueTarget(joints_ik);
  
    // Plan trajectory
  if (!group_->plan(next_plan_)){
    group_->clearPathConstraints();
      ROS_INFO("Motion planning to position (%.2f, %.2f, %.2f) failed", 
      pose.position.x, pose.position.y, pose.position.z);
    return false;
  }
  group_->clearPathConstraints();
  ROS_INFO("Motion planning to position (%.2f, %.2f, %.2f) successful", 
      pose.position.x, pose.position.y, pose.position.z);

  // Execute trajectory
  if (executeJointTrajectory(next_plan_)) {
    ROS_INFO("Trajectory execution successful");
    return true;
  }
  else {
    ROS_ERROR("Trajectory execution failed");
    return false;
  }
  
}

moveit_msgs::CollisionObjectPtr PegInHole::getCollisionObject(std::string object_name)
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

bool PegInHole::attachObject(std::string object_name){ 

  moveit_msgs::CollisionObjectPtr coll_obj = getCollisionObject(object_name);
  if (coll_obj) {
    ROS_INFO_STREAM("Attaching object "<<object_name<<" to the end-effector");
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = ee_frame_;
    attached_object.object = *coll_obj;
    attached_object.object.operation = attached_object.object.ADD;
    attached_object_publisher_.publish(attached_object);
    return true;
  } else 
    return false;
  
}

bool PegInHole::detachObject(){
  ROS_INFO_STREAM("Detaching object from the robot");

  // update the planning scene to get the robot's state
  getPlanningScene(planning_scene_msg_, full_planning_scene_);

  if (planning_scene_msg_.robot_state.attached_collision_objects.size()>0){
    moveit_msgs::AttachedCollisionObject attached_object = planning_scene_msg_.robot_state.attached_collision_objects[0];
    geometry_msgs::Pose object_pose;
    this->compute_fk(planning_scene_msg_.robot_state.joint_state, object_pose);

    // TODO
    // Translation between /link_7 and /ati_link
    object_pose.position.z -= 0.055;
    
    tf::Quaternion co_quat;
    tf::quaternionMsgToTF(object_pose.orientation, co_quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(co_quat).getRPY(roll, pitch, yaw);
    tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,yaw);
    
    attached_object.object.header.frame_id = "base_link";
    
    if (attached_object.object.primitive_poses.size() >0){

    attached_object.object.primitive_poses[0].position = object_pose.position;
    attached_object.object.primitive_poses[0].orientation.x = quat.x();
    attached_object.object.primitive_poses[0].orientation.y = quat.y();
    attached_object.object.primitive_poses[0].orientation.z = quat.z();
    attached_object.object.primitive_poses[0].orientation.w = quat.w();
    }else{
      attached_object.object.mesh_poses[0].position = object_pose.position;
      attached_object.object.mesh_poses[0].orientation.x = quat.x();
      attached_object.object.mesh_poses[0].orientation.y = quat.y();
      attached_object.object.mesh_poses[0].orientation.z = quat.z();
      attached_object.object.mesh_poses[0].orientation.w = quat.w();
    }
    
    planning_scene_msg_.robot_state.attached_collision_objects.clear();
    planning_scene_msg_.world.collision_objects.push_back(attached_object.object);
    planning_scene_msg_.is_diff = true;
    planning_scene_diff_publisher_.publish(planning_scene_msg_);
    return true;
  }else{
    ROS_ERROR("There was no object attached to the robot");
    return false;
  }
}

void PegInHole::cleanObjects(){
  // update the planning scene to get the robot's state
  getPlanningScene(planning_scene_msg_, full_planning_scene_);
  
  for (int i = 0; i<planning_scene_msg_.robot_state.attached_collision_objects.size(); i++)
    planning_scene_msg_.robot_state.attached_collision_objects[i].object.operation = moveit_msgs::CollisionObject::REMOVE;
  for (int i = 0; i<planning_scene_msg_.world.collision_objects.size(); i++)
    planning_scene_msg_.world.collision_objects[i].operation = moveit_msgs::CollisionObject::REMOVE;
  
  planning_scene_msg_.is_diff = true;
  planning_scene_diff_publisher_.publish(planning_scene_msg_);
}

bool PegInHole::moveAboveObjectHole(const std::string obj_name, const int hole_nb)
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
  up_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.18));
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

bool PegInHole::loadHolesLocation(const std::string obj_name)
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
