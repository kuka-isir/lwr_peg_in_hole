#include <lwr_peg_in_hole/robot_move.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo1");
  
  RobotMove robot_move;
  
  std::cout << "\n \n \n Waiting for input ..... Press ENTER ! \n \n \n ";
  std::cin.ignore();
  
  robot_move.loadHolesLocation("plaque2");

  while(ros::ok()){
    while(!robot_move.moveToStart() && ros::ok()){};

    while(!robot_move.moveAboveObjectHole("plaque2", 0) && ros::ok()){};

    while(!robot_move.moveToStart() && ros::ok()){};

    while(!robot_move.moveAboveObjectHole("plaque2", 1) && ros::ok()){};
  }

  ros::shutdown();
  return 0;
}