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


  while(!robot_move.moveToStart() && ros::ok()){usleep(1E6);}
  while(!robot_move.moveToJointPosition(look_up1) && ros::ok()){usleep(1E6);}

  usleep(2E6);
  
  while(!up_client.call(req_update,resp_update) && ros::ok())
  {
      usleep(1E6);
      ROS_INFO("Waiting for object");
  }

  while(!estimate_client.call(req_estimate,resp_estimate) && ros::ok())
  {
      usleep(1E6);
      ROS_INFO("Getting the holes pose estimate");
  }
  
  robot_move.loadHolesLocation("plaque2");
  while(!robot_move.moveAboveObjectHole("plaque2", 0) && ros::ok()){usleep(1E6);}
  

//   geometry_msgs::Pose above_hole = resp_estimate.holes_poses[0];
//   above_hole.position.z += 0.45;
//   above_hole.orientation.x = 1.0;
//   above_hole.orientation.y = 0;
//   above_hole.orientation.z = 0;
//   above_hole.orientation.w = 0;
//   
//   sensor_msgs::JointState above_hole_jnts;
//   robot_move.compute_ik(above_hole,above_hole_jnts);
//   std::vector<double> above_hole_jnts_vector;
//   
//   for(int i=0; i<above_hole_jnts.position.size();i++){
//     above_hole_jnts_vector.push_back(above_hole_jnts.position[i]);
//   }
//   std::cout<<"Pose \n" <<above_hole <<std::endl;
//   
//   std::cout<<"Joints \n" <<above_hole_jnts <<std::endl;
//   while(!robot_move.moveToJointPosition(above_hole_jnts_vector) && ros::ok()){usleep(1E6);}



    geometry_msgs::Pose test_lin_rel;
    test_lin_rel.orientation.w = 1.0;
    test_lin_rel.position.z = -0.08;
    robot_move.moveLinRel(test_lin_rel);


  ros::shutdown();
  return 0;
}