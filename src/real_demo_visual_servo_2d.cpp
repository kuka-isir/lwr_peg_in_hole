#include <lwr_peg_in_hole/robot_move.hpp>
#include <unistd.h>
#include <geometry_msgs/Twist.h>

class Servo2d
{
public:
  Servo2d(ros::NodeHandle& nh,bool is_sim):nh_(nh),robot_move(is_sim)
  {
     sub_error_ = nh_.subscribe("/ellipse_detection/error",1,&Servo2d::callback,this);
  }
  void callback(const geometry_msgs::TwistConstPtr& error_ptr)
  {
  
//     geometry_msgs::PoseStamped error_in_cam_msg,error_in_link_0_msg;
//     error_in_cam_msg.header.frame_id = "head_cam_link_optical";
//     error_in_cam_msg.pose.orientation.w = 1.0;
//     error_in_cam_msg.pose.position.x = error_ptr->linear.x;
//     error_in_cam_msg.pose.position.y = error_ptr->linear.y;
// 
//     
//     ROS_INFO_STREAM("Error : " << *error_ptr);
//     // Error is in camera frame TODO: Listen to the cam_info topic
//     
//     try
//     {
// //       tf_listener_.waitForTransform("link_0","head_cam_link_optical", ros::Time(0.0), ros::Duration(1.0));
//       std::string s;
//       ros::Time t;
//       tf_listener_.getLatestCommonTime("head_cam_link_optical","screwdriver_link",t,&s);
//       error_in_cam_msg.header.stamp = t;
//       tf_listener_.transformPose("screwdriver_link",error_in_cam_msg,error_in_link_0_msg);
//     }
//     catch(tf::TransformException& e)
//     {
//       ROS_ERROR_STREAM(""<<e.what());
//       return;
//     }
    
//       ROS_INFO_STREAM("Error in screwdriver_link : "<<error_in_link_0_msg);
    
    
    geometry_msgs::Pose rel;
    rel.orientation.w = 1.0;
    if(error_ptr->linear.x > 0)
      rel.position.x = std::min(error_ptr->linear.x*0.0001,0.01);
    else
      rel.position.x = std::max(error_ptr->linear.x*0.0001,-0.01);
    
    if(error_ptr->linear.y > 0)
      rel.position.y = std::min(error_ptr->linear.y*0.0001,0.01);
    else
      rel.position.y = std::max(error_ptr->linear.y*0.0001,-0.01);
    
    robot_move.moveLinRelInTool(rel);
  }
  ros::NodeHandle nh_;
  ros::Subscriber sub_error_;
  RobotMove robot_move;
  tf::TransformListener tf_listener_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_servo");
  ros::NodeHandle nh("");

  bool is_sim = false;
  Servo2d s(nh,is_sim);
  
  
//   std::cout << "\n \n \n Waiting for input ..... Press ENTER ! \n \n \n ";
//   std::cin.ignore();
  
  ros::spin();
  ros::shutdown();
  return 0;
}