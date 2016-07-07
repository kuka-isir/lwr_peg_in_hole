#include <lwr_peg_in_hole/object_spawn.hpp>

ObjectSpawn::ObjectSpawn() : spinner_(1)
{
  ros::WallDuration sleep_t(0.5);
  spinner_.start();
  planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher_.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
}

bool ObjectSpawn::updateObjectInScene(std::string& object_name, int& tag_id){

  std::string tag_name = "/ar_marker_"+ boost::to_string(tag_id);

  ROS_INFO("Calling update in scene for object %s with tag %d", object_name.c_str(), tag_id);

  // Check if mesh exists
  std::string path_to_mesh = ros::package::getPath("lwr_peg_in_hole")+"/meshes/"+object_name+".stl";
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
  shapes::Mesh* m = shapes::createMeshFromResource("package://lwr_peg_in_hole/meshes/"+object_name+".stl");
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

  // Define the object's position //
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

  geometry_msgs::Pose mesh_pose;
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
  moveit_msgs::PlanningScene planning_scene_msg_;
  planning_scene_msg_.world.collision_objects.push_back(collision_object);
  planning_scene_msg_.is_diff = true;
  planning_scene_diff_publisher_.publish(planning_scene_msg_);

  return true;
};