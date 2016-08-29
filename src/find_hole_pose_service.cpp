#include <lwr_peg_in_hole/find_hole_pose_service.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_hole_pose_service");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  FindHolePoseService hole_service;
  ros::spin();
  
  ros::shutdown();
  return 0;
}