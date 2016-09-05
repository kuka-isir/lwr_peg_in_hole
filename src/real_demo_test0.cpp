#include <lwr_peg_in_hole/robot_move.hpp>
#include <unistd.h>
#include <lwr_peg_in_hole/UpdateSceneService.h>
#include <lwr_peg_in_hole/EstimateHolesService.h>
#include <lwr_peg_in_hole/FindHolePoseService.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo0");
  ros::NodeHandle nh("");


  bool sim = false;
  RobotMove robot_move(sim);

  std::cout << "\n \n \n Waiting for input ..... Press ENTER ! \n \n \n ";
  std::cin.ignore();

  ros::ServiceClient up_client = nh.serviceClient<lwr_peg_in_hole::UpdateSceneService>("update_scene");
  ros::ServiceClient estimate_client = nh.serviceClient<lwr_peg_in_hole::EstimateHolesService>("estimate_holes");
  ros::ServiceClient find_client = nh.serviceClient<lwr_peg_in_hole::FindHolePoseService>("find_hole_pose");
  while(!up_client.exists() && !estimate_client.exists() && !find_client.exists())
  {
    ROS_INFO("Waiting for update_scene, estimate_holes and find_hole_pose services...");
    sleep(1.0);
  }

  ROS_INFO("Services ready !");
  lwr_peg_in_hole::UpdateSceneServiceRequest req_update;
  lwr_peg_in_hole::UpdateSceneServiceResponse resp_update;
  req_update.object_name = "plaque2";
  req_update.tag_id = 100;

  lwr_peg_in_hole::EstimateHolesServiceRequest req_estimate;
  lwr_peg_in_hole::EstimateHolesServiceResponse resp_estimate;
  req_estimate.object_name = "plaque2";
  
  lwr_peg_in_hole::FindHolePoseServiceRequest req_find;
  lwr_peg_in_hole::FindHolePoseServiceResponse resp_find;
  
  std::vector<double> look_up1, look_up2, look_up3;
  look_up1.push_back(1.64647737821113);
  look_up1.push_back(-0.06401549522493077);
  look_up1.push_back(-0.37189732272924836);
  look_up1.push_back(-1.7298683559749657);
  look_up1.push_back(-0.010311991928772635);
  look_up1.push_back(1.2837615843359318);
  look_up1.push_back(-1.5951452987645744);

  look_up2.push_back(1.7450482624189192);
  look_up2.push_back(-0.2792615169621282);
  look_up2.push_back(-0.7326299417794155);
  look_up2.push_back(-1.6953566682827867);
  look_up2.push_back(-0.1489061268374794);
  look_up2.push_back(1.5780836265219742);
  look_up2.push_back(-1.8916098999458724);

  look_up3.push_back(1.9830371040607986);
  look_up3.push_back(-0.0006218652097080835);
  look_up3.push_back(0.009821608276626925);
  look_up3.push_back(-1.7048954646128616);
  look_up3.push_back(-0.0576423149512717);
  look_up3.push_back(1.3746182539883787);
  look_up3.push_back(-0.910279200952389);

  /*********** DEMO script *****************/
  // Go to start position
  while(!robot_move.moveToStart() && ros::ok()){usleep(1E6);}
  
  // Go to lookup position
  while(!robot_move.moveToJointPosition(look_up1) && ros::ok()){usleep(1E6);}

  // Wait 2s
  usleep(2E6);
  
  // Update object position in moveit planning scene
  while(!up_client.call(req_update,resp_update) && ros::ok())
  {
      usleep(1E6);
      ROS_INFO("Waiting for object...");
  }
  
  // Read holes poses
  robot_move.loadHolesLocation("plaque2");
  while(!estimate_client.call(req_estimate,resp_estimate) && ros::ok())
  {
      usleep(1E6);
      ROS_INFO("Getting the holes pose estimate...");
  }
  
  // Move to above first hole
  while(!robot_move.moveAboveObjectHole("plaque2", 0) && ros::ok()){usleep(1E6);}
  
  // Get closer to the hole
  geometry_msgs::Pose test_lin_rel;
  test_lin_rel.orientation.w = 1.0;
  test_lin_rel.position.z = -robot_move.dist_above_hole_/2.0;
  robot_move.moveLinRel(test_lin_rel);
 
  // Wait 2s to stabilize camera image
  usleep(2E6);
  
  // Update object position in moveit planning scene
  req_find.hole_pose_estimate = resp_estimate.holes_poses[0];
  while(!find_client.call(req_find,resp_find) && ros::ok())
  {
      usleep(1E6);
      ROS_INFO("Looking for real hole pose...");
  }
  std::cout << "Estimate pose\n" << req_find.hole_pose_estimate << std::endl;
  std::cout << "Correct pose\n" << resp_find.hole_pose << std::endl;
  
  
  // Correct pose
  resp_find.hole_pose.position.z += robot_move.dist_above_hole_/2.0;
  
  tf::Transform hole_transform;
  hole_transform.setOrigin(tf::Vector3(resp_find.hole_pose.position.x, resp_find.hole_pose.position.y, resp_find.hole_pose.position.z));
  hole_transform.setRotation(tf::Quaternion(resp_find.hole_pose.orientation.x, resp_find.hole_pose.orientation.y, resp_find.hole_pose.orientation.z, resp_find.hole_pose.orientation.w));

  tf::Transform pi_rotation_transform;
  pi_rotation_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion pi_rotation;
  pi_rotation.setRPY(M_PI,0,0);
  pi_rotation_transform.setRotation(pi_rotation);

  hole_transform *= pi_rotation_transform;
  
  resp_find.hole_pose.position.x = hole_transform.getOrigin().getX();
  resp_find.hole_pose.position.y = hole_transform.getOrigin().getY();
  resp_find.hole_pose.position.z = hole_transform.getOrigin().getZ();
//   resp_find.hole_pose.orientation.x = hole_transform.getRotation().getX();
//   resp_find.hole_pose.orientation.y = hole_transform.getRotation().getY();
//   resp_find.hole_pose.orientation.z = hole_transform.getRotation().getZ();
//   resp_find.hole_pose.orientation.w = hole_transform.getRotation().getW();
  
    // HACK !!!!!!!!!!
  tf::Quaternion perfect_orientation;
  perfect_orientation.setRPY(-M_PI,0,-(M_PI/2.0 + 20*M_PI/180.0));
  resp_find.hole_pose.orientation.x = perfect_orientation.x();
  resp_find.hole_pose.orientation.y = perfect_orientation.y();
  resp_find.hole_pose.orientation.z = perfect_orientation.z();
  resp_find.hole_pose.orientation.w = perfect_orientation.w();
  
  
  while(!robot_move.moveToCartesianPose(resp_find.hole_pose) && ros::ok()){usleep(1E6);}
  
  ros::shutdown();
  return 0;
}