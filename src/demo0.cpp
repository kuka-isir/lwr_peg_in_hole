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

  std::cout << "1) Move to start...";
  while(!robot_move.moveToStart() && ros::ok())
  {
    usleep(1E6);
  }
  std::cout << "...done" << std::endl;
// -0.6065080165863037, 0.3229283094406128, -0.2579362690448761, -1.7599897384643555, 0.03919009491801262, 1.0888139009475708, -1.9234648942947388
  std::vector<double> first_fastener;
  first_fastener.push_back(-0.6065080165863037);
  first_fastener.push_back(0.3229283094406128);
  first_fastener.push_back(-0.2579362690448761);
  first_fastener.push_back(-1.7599897384643555);
  first_fastener.push_back(0.03919009491801262);
  first_fastener.push_back(1.0888139009475708);
  first_fastener.push_back(-1.9234648942947388);
// -0.6065362095832825, 0.564785897731781, -0.2343180775642395, -1.4194331169128418, 0.0830741822719574, 1.2019346952438354, -1.895105004310608
  std::vector<double> second_fastener;
  second_fastener.push_back(-0.6065362095832825);
  second_fastener.push_back(0.564785897731781);
  second_fastener.push_back(-0.2343180775642395);
  second_fastener.push_back(-1.4194331169128418);
  second_fastener.push_back(0.0830741822719574);
  second_fastener.push_back(1.2019346952438354);
  second_fastener.push_back(-1.895105004310608);


  while(ros::ok()){
    std::cout << "2) Move to start";
    while(!robot_move.moveToStart() && ros::ok())
    {
      usleep(1E6);
    }
    std::cout << "...done" << std::endl;

    std::cout << "3) Move to first_fastener";
    while(!robot_move.moveToJointPosition(first_fastener) && ros::ok())
    {
        usleep(1E6);
    }
    std::cout << "...done" << std::endl;

    usleep(1E6);

    while(!robot_move.moveToStart() && ros::ok())
    {
      usleep(1E6);
    }

    while(!robot_move.moveToJointPosition(second_fastener) && ros::ok()){usleep(1E6);}

    usleep(1E6);

    while(!robot_move.moveToStart() && ros::ok()){usleep(1E6);}

    geometry_msgs::Pose test_pose;
    test_pose.position.x = -0.161;
    test_pose.position.y = -0.209;
    test_pose.position.z = 0.866;
    test_pose.orientation.x = -0.596;
    test_pose.orientation.y = 0.644;
    test_pose.orientation.z = -0.236;
    test_pose.orientation.w = -0.418;

    while(!robot_move.moveToCartesianPose(test_pose) && ros::ok()){usleep(1E6);}
    
    while(!robot_move.moveToStart() && ros::ok()){usleep(1E6);}
    
    geometry_msgs::Pose test_lin_rel;
    test_lin_rel.orientation.w = 1.0;
    test_lin_rel.position.z = 0.1;
    while(!robot_move.moveLinRel(test_lin_rel) && ros::ok()){usleep(1E6);}

    usleep(1E6);

  }

  ros::shutdown();
  return 0;
}
