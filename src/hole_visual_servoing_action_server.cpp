#include <actionlib/server/simple_action_server.h>
#include <lwr_peg_in_hole/HoleVisualServoingAction.h>
#include <lwr_peg_in_hole/hole_visual_servoing.hpp>
#include <lwr_peg_in_hole/robot_move.hpp>

class HoleServoingActionServer
{
public:
  HoleServoingActionServer(ros::NodeHandle& nh, HoleVisualServoing& hole_visual_servoing, RobotMove& robot_move) :
    nh_(nh), hole_visual_servoing_(hole_visual_servoing), robot_move_(robot_move),
    act_srv_(nh, "hole_visual_servoing", boost::bind(&HoleServoingActionServer::executeCB, this, _1), false)
  {
    act_srv_.start();
  }

protected:
  ros::NodeHandle& nh_;
  HoleVisualServoing& hole_visual_servoing_;
  RobotMove& robot_move_;

  actionlib::SimpleActionServer<lwr_peg_in_hole::HoleVisualServoingAction> act_srv_;
  lwr_peg_in_hole::HoleVisualServoingFeedback feedback_;
  lwr_peg_in_hole::HoleVisualServoingResult result_;

  void executeCB(const lwr_peg_in_hole::HoleVisualServoingGoalConstPtr& goal)
  {
    if(goal->save_hole){
      hole_visual_servoing_.save_ideal_hole_location_ = true;
      result_.success = true;
      act_srv_.setSucceeded(result_);
      return;
    }
    
    ros::Rate r(30);
    while (ros::ok()) {
      if(hole_visual_servoing_.isPinAboveHole()) {
        ROS_INFO("HoleVisualServoing action complete.");
        result_.success = true;
        act_srv_.setSucceeded(result_);
        return;
      }
      
      if(hole_visual_servoing_.has_set_ideal_){
        geometry_msgs::Pose rel;
        rel.orientation.w = 1.0;
        if(hole_visual_servoing_.error_x_ > 0)
          rel.position.x = std::min(hole_visual_servoing_.error_x_*0.0001,0.01);
        else
          rel.position.x = std::max(hole_visual_servoing_.error_x_*0.0001,-0.01);
        
        if(hole_visual_servoing_.error_y_ > 0)
          rel.position.y = std::min(hole_visual_servoing_.error_y_*0.0001,0.01);
        else
          rel.position.y = std::max(hole_visual_servoing_.error_y_*0.0001,-0.01);
      
        std::cout << "Sending command ["<<rel.position.x<<","<<rel.position.y<<"]"<<std::endl;
        robot_move_.moveLinRelInTool(rel,0.1);
      }
      else{
        ROS_ERROR("Ideal hole location not saved earlier... !");
      }
      
      ros::spinOnce();
      r.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hole_visual_servoing_action_server");

  ros::NodeHandle nh;
  RobotMove robot_move(false);
  
  HoleVisualServoing hole_visual_servoing(nh);

  ROS_WARN("hole_visual_servoing ready");
  
  HoleServoingActionServer hole_visual_servoing_as(nh, hole_visual_servoing, robot_move);
  ros::spin();
  ros::shutdown();
  return 0;
}