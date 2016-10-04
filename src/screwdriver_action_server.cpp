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
    pub_filter_val_ = nh_.advertise<std_msgs::Float64>("/screwdriver/effort_controller/filter", 1);
    stopping_cmd_.data = 0.0;
    opening_cmd_.data = -10000.0;
    closing_cmd_.data = 10000.0;
    ROS_INFO("screwdriver action server ready !");
    
    alpha_ = 0.95;
    time_min_open_ = 20.0;
    time_min_close_ = 20.0;
    time_max_ = 35.0;
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_cmd_, pub_filter_val_;
  double current_effort_, filtered_val_, alpha_, time_min_open_, time_min_close_, time_max_;
  
  std_msgs::Float64 opening_cmd_, closing_cmd_, stopping_cmd_;

  actionlib::SimpleActionServer<lwr_peg_in_hole::ScrewdriverAction> act_srv_;
  lwr_peg_in_hole::ScrewdriverFeedback feedback_;
  lwr_peg_in_hole::ScrewdriverResult result_;

  void executeCB(const lwr_peg_in_hole::ScrewdriverGoalConstPtr& goal)
  {
    ros::Subscriber effort_sub_ = nh_.subscribe("/screwdriver/joint_states", 1, &ScrewdriverActionServer::effortCB,this);
    filtered_val_ = 0.0;
    
    /** Opening fastener **/
    if(goal->opening){
      
      pub_cmd_.publish(opening_cmd_);
      
      
      ros::Time begin = ros::Time::now();
      ros::Duration spent = ros::Duration(0.0);
      while((ros::ok() && spent.toSec()<time_min_open_ )
            || (ros::ok() && spent.toSec()<time_max_ && filtered_val_ > -3.8 ) ){
        // Closing for 1s
        spent = ros::Time::now() - begin;
        usleep(1000*1000*1);
        ros::spinOnce();
      }
      
      pub_cmd_.publish(stopping_cmd_);
      
      result_.success = true;
      act_srv_.setSucceeded(result_);
      return;
      
    /** Closing fastener **/
    }else{
      pub_cmd_.publish(closing_cmd_);
      
      ros::Time begin = ros::Time::now();
      ros::Duration spent = ros::Duration(0.0);
      while((ros::ok() && spent.toSec()<time_min_close_ )
            || (ros::ok() && spent.toSec()<time_max_ && filtered_val_ < 9.5 ) ){
        // Closing for 1s
        spent = ros::Time::now() - begin;
        usleep(1000*1000*1);
        ros::spinOnce();
      }
      
      // Closing for 8s
      usleep(1000*1000*8);
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
    filtered_val_ = filtered_val_*alpha_ + current_effort_*(1-alpha_);
    std_msgs::Float64 filter;
    filter.data = filtered_val_;
    pub_filter_val_.publish(filter);
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