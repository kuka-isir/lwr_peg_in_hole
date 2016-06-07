#include <lwr_peg_in_hole/peg_in_hole.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "peg_in_hole_node");

//   // Define pose for depose fo the epingle
//   geometry_msgs::Pose depose_pose;
//   depose_pose.position.x = 0.5;
//   depose_pose.position.y = 0.0;
//   depose_pose.position.z = 0.2;
//   tf::Quaternion tf_quat;
//   tf_quat.setRPY(M_PI, 0.0, 0.0);
//   geometry_msgs::Quaternion geo_quat;
//   tf::quaternionTFToMsg(tf_quat, geo_quat);
//   depose_pose.orientation = geo_quat;
//   
//   // Define init pose for the epingle
//   geometry_msgs::Pose epingle_pose;
//   epingle_pose.position.x = 0.5;
//   epingle_pose.position.y = 0.0;
//   epingle_pose.position.z = 0.12;
//   tf::Quaternion q1;
//   q1.setRPY(-M_PI/4.0, 0.0, 0.0);
//   geometry_msgs::Quaternion quat1;
//   tf::quaternionTFToMsg(q1, quat1);  
//   epingle_pose.orientation = quat1;
  
//   // Define init pose for the plaque
//   geometry_msgs::Pose plaque_pose;
//   tf::Quaternion q;
//   q.setRPY(-M_PI/2.0+M_PI/4.0, M_PI/4.0, -M_PI/2.0);
//   geometry_msgs::Quaternion quat;
//   tf::quaternionTFToMsg(q, quat);  
//   plaque_pose.orientation = quat;
//   plaque_pose.position.x = 0.8;
//   plaque_pose.position.z = 0.5;
  
  
  PegInHole pick_n_place;
//   pick_n_place.cleanObjects();
  usleep(1000000*1);
  
//   // Set the setup 
//   pick_n_place.addEpingleObject(epingle_pose);
//   pick_n_place.addPlaqueObject(plaque_pose);

  // First: demo with set up already in place
  pick_n_place.moveToStart();
//   pick_n_place.moveAboveEpingle("epingle");
//   pick_n_place.moveToEpingle("epingle");
//   pick_n_place.attachObject("epingle");
  pick_n_place.loadHolesLocation("holes_board");
//   pick_n_place.moveAbovePlaque("holes_board");
  pick_n_place.moveAboveObjectHole("holes_board", 0);
  pick_n_place.moveAboveObjectHole("holes_board", 1);
  pick_n_place.moveAboveObjectHole("holes_board", 2);
  pick_n_place.moveAboveObjectHole("holes_board", 3);
//   pick_n_place.moveToPlaque("plaque");
//   pick_n_place.moveToCartesianPose(depose_pose);
//   pick_n_place.detachObject();
//   pick_n_place.moveToStart();
  
  // Keep going if user wants to move the set up
//   int run_prg = 1, first =1;
//   std::cout << "run more ? 0/1" <<std::endl;
//   std::cin >> run_prg;
//   while(run_prg && ros::ok()){
//     
//     pick_n_place.moveAboveEpingle("epingle");
//     pick_n_place.moveToEpingle("epingle");
//     pick_n_place.attachObject("epingle");
//     pick_n_place.moveAbovePlaque("plaque");
//     pick_n_place.moveToPlaque("plaque");
//     pick_n_place.moveToCartesianPose(depose_pose);
//     pick_n_place.detachObject();
//     pick_n_place.moveToStart();
//     
//     std::cout << "run more? 0/1" <<std::endl;
//     std::cin >> run_prg;
//   }
  ros::shutdown();
  return 0;
}