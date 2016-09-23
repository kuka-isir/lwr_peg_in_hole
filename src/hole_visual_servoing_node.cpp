#include <lwr_peg_in_hole/hole_visual_servoing.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hole_visual_servoing");

  ros::NodeHandle nh;
  
  HoleVisualServoing hole_visual_servoing(nh);
  
  ros::spin();
  ros::shutdown();
  return 0;
}