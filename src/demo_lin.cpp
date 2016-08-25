#include <lwr_peg_in_hole/robot_move.hpp>
#include <unistd.h>
#include <lwr_peg_in_hole/UpdateSceneService.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo0");
  ros::NodeHandle nh("");


  bool sim = false;
  RobotMove robot_move(sim);

  std::cout << "\n \n \n Waiting for input ..... Press ENTER ! \n \n \n ";
  std::cin.ignore();

  std::vector<double> first_fastener;
  first_fastener.push_back(-0.6065080165863037);
  first_fastener.push_back(0.3229283094406128);
  first_fastener.push_back(-0.2579362690448761);
  first_fastener.push_back(-1.7599897384643555);
  first_fastener.push_back(0.03919009491801262);
  first_fastener.push_back(1.0888139009475708);
  first_fastener.push_back(-1.9234648942947388);

  while(ros::ok()){

    while(!robot_move.moveToStart() && ros::ok()){usleep(1E6);}

    geometry_msgs::Pose test_lin_rel;
    test_lin_rel.orientation.w = 1.0;
    test_lin_rel.position.z = 0.1;
    while(!robot_move.moveLinRel(test_lin_rel) && ros::ok()){usleep(1E6);}
    test_lin_rel.position.z = 0.0;
    test_lin_rel.position.y = -0.1;
    while(!robot_move.moveLinRel(test_lin_rel) && ros::ok()){usleep(1E6);}
    test_lin_rel.position.z = -0.35;
    test_lin_rel.position.y = 0.0;
    while(!robot_move.moveLinRel(test_lin_rel) && ros::ok()){usleep(1E6);}
    test_lin_rel.position.z = 0.0;
    test_lin_rel.position.y = 0.01;
    while(!robot_move.moveLinRel(test_lin_rel) && ros::ok()){usleep(1E6);}
    while(!robot_move.moveLinRel(test_lin_rel) && ros::ok()){usleep(1E6);}
    while(!robot_move.moveLinRel(test_lin_rel) && ros::ok()){usleep(1E6);}

  }

  ros::shutdown();
  return 0;
}
