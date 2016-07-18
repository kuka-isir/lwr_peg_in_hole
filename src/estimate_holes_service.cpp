#include <lwr_peg_in_hole/estimate_holes_service.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimate_holes_service");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  EstimateHolesService estimate_holes_service;
  ros::spin();
  
  ros::shutdown();
  return 0;
}