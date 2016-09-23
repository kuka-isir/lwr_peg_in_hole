#include <actionlib/server/simple_action_server.h>
#include <lwr_peg_in_hole/ScrewdriverAction.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

class ScrewdriverActionServer
{
public:
  ScrewdriverActionServer(ros::NodeHandle& nh) :
    nh_(nh),
    act_srv_(nh, "screwdriver", boost::bind(&ScrewdriverActionServer::executeCB, this, _1), false)
  {
    act_srv_.start();
    pub_cmd_ = nh_.advertise<std_msgs::Float64>("/screwdriver/effort_controller/command", 1);
    stopping_cmd_.data = 0.0;
    opening_cmd_.data = -10000.0;
    closing_cmd_.data = 10000.0;
    ROS_INFO("screwdriver action server ready !");
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_cmd_;
  double current_effort_;
  
  std_msgs::Float64 opening_cmd_, closing_cmd_, stopping_cmd_;

  actionlib::SimpleActionServer<lwr_peg_in_hole::ScrewdriverAction> act_srv_;
  lwr_peg_in_hole::ScrewdriverFeedback feedback_;
  lwr_peg_in_hole::ScrewdriverResult result_;

  void executeCB(const lwr_peg_in_hole::ScrewdriverGoalConstPtr& goal)
  {
    ros::Subscriber effort_sub_ = nh_.subscribe("/screwdriver/joint_states", 1, &ScrewdriverActionServer::effortCB,this);
    
    /** Opening fastener **/
    if(goal->opening){
      
      pub_cmd_.publish(opening_cmd_);
      
      // Opening for 45s
      usleep(1000*1000*45);
      
      pub_cmd_.publish(stopping_cmd_);
      
      result_.success = true;
      act_srv_.setSucceeded(result_);
      return;
      
    /** Closing fastener **/
    }else{
      pub_cmd_.publish(closing_cmd_);
      
      // Closing for 1s
      usleep(1000*1000*1);
      ros::spinOnce();
      
      while(ros::ok() && current_effort_ < 9.0){
        // Closing for 1s
        usleep(1000*1000*1);
        ros::spinOnce();
      }
      
      // Closing for 5s
      usleep(1000*1000*5);
      ros::spinOnce();
      
      pub_cmd_.publish(stopping_cmd_);
      result_.success = true;
      act_srv_.setSucceeded(result_);
      
      return;
    }
  }

  void effortCB(const sensor_msgs::JointState::ConstPtr& msg)
  {
    current_effort_ = msg->effort[0];
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "screwdriver_control_action_server");
  usleep(1000*1000);

  ros::NodeHandle nh;
  ScrewdriverActionServer screwdriver_as(nh);
  
  ros::spin();
  ros::shutdown();
  return 0;
}