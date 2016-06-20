#include <lwr_peg_in_hole/update_scene_service.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "update_scene_service");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  UpdateSceneService scene_service;
  ros::spin();
  
  ros::shutdown();
  return 0;
}

