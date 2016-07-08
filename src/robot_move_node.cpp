#include <lwr_peg_in_hole/robot_move.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_move_action_server");
  usleep(1000*1000);

  ros::NodeHandle nh;
  RobotMove robot_move;
  std::vector<double> joint_vals;
  joint_vals.push_back(1.6002605785960622);
  joint_vals.push_back(-0.0449239937131054);
  joint_vals.push_back(-0.333517572028704);
  joint_vals.push_back(-1.707667848031405);
  joint_vals.push_back(-4.734100141323694e-05);
  joint_vals.push_back(1.3883824216456677);
  joint_vals.push_back(-0.00020880873042017356);
  robot_move.moveToJointPosition(joint_vals);
  
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.369;
  target_pose.position.y = 0.078;
  target_pose.position.z = 0.410;
  target_pose.orientation.x = 0.993;
  target_pose.orientation.y = 0.110;
  target_pose.orientation.z = 0.036;
  target_pose.orientation.w = -0.011;
  robot_move.moveToCartesianPose(target_pose);

  ros::shutdown();
  return 0;
}