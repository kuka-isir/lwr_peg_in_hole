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

  while(ros::ok()){

    while(!robot_move.moveToStart() && ros::ok()){usleep(1E6);}

    geometry_msgs::Vector3 XYZ_mm,ABC_deg;
    XYZ_mm.x = -399.96;
    XYZ_mm.y = -261.67;
    XYZ_mm.z = 102.15;

    ABC_deg.x = -90.0;
    ABC_deg.y = 0.0;
    ABC_deg.z = 180.0;

    while(!robot_move.moveToCartesianPoseUsingPTP_KUKA_Conventions(XYZ_mm,ABC_deg) && ros::ok()){usleep(1E6);}

    geometry_msgs::Pose lin_cmd;
    lin_cmd.orientation.w = 1.0;
    lin_cmd.position.z = -0.1;

    while(!robot_move.moveLinRel(lin_cmd) && ros::ok()){usleep(1E6);}

    lin_cmd.position.z = 0.1;

    while(!robot_move.moveLinRel(lin_cmd) && ros::ok()){usleep(1E6);}

    // while(!robot_move.moveToStart() && ros::ok()){usleep(1E6);}
    //
    // geometry_msgs::Pose test_lin_rel;
    // test_lin_rel.orientation.w = 1.0;
    // test_lin_rel.position.z = 0.1;
    // while(!robot_move.moveToCartesianPoseUsingPTP(test_lin_rel,true) && ros::ok()){usleep(1E6);}

  }

  ros::shutdown();
  return 0;
}
