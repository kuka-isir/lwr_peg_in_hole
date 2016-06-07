#include <lwr_peg_in_hole/spawn_holes.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_object");
  ros::NodeHandle nh, nh_param("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  double x_goal, y_goal, z_goal, roll, pitch, yaw;

  // Load the planning_scene and make sure we can publish the scene //
  moveit_msgs::PlanningScene planning_scene;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::WallDuration sleep_t(0.5);
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }

  // Define the attached object message //
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = "holes_board";

  // Define the mesh //
  shapes::Mesh* m = shapes::createMeshFromResource("package://lwr_peg_in_hole/meshes/holes_board.stl");

  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

  // Define the object's position //
  geometry_msgs::Pose mesh_pose;
  if(nh_param.getParam("x_goal",x_goal)){
    mesh_pose.position.x = x_goal;
  }
  if(nh_param.getParam("y_goal",y_goal)){
    mesh_pose.position.y = y_goal;
  }
  if(nh_param.getParam("z_goal",z_goal)){
    mesh_pose.position.z = z_goal;
  }
  if(nh_param.getParam("roll",roll) && nh_param.getParam("pitch",pitch) && nh_param.getParam("yaw",yaw)){
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll,pitch,yaw);
    mesh_pose.orientation.x = quat.x();
    mesh_pose.orientation.y = quat.y();
    mesh_pose.orientation.z = quat.z();
    mesh_pose.orientation.w = quat.w();
  }

  // Give info back
  std::ostringstream info;
  info << "Adding an object at pose :\n" << "x = "<<mesh_pose.position.x<<" ; y = "<<mesh_pose.position.y<<" ; z = "<<mesh_pose.position.z;
  ROS_INFO_STREAM(info.str());

  // Attach object operation //
  collision_object.meshes.push_back(co_mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_object.operation = collision_object.ADD;

  // Put the object in the environment //
  planning_scene.world.collision_objects.push_back(collision_object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  ros::shutdown();
}