#include <actionlib/server/simple_action_server.h>
#include <lwr_peg_in_hole/RobotMoveAction.h>
#include <lwr_peg_in_hole/robot_move.hpp>

class RobotMoveActionServer
{
public:
  RobotMoveActionServer(ros::NodeHandle& nh, RobotMove& robot_move) :
    nh_(nh), robot_move_(robot_move),
    act_srv_(nh, "robot_mover", boost::bind(&RobotMoveActionServer::executeCB, this, _1), false)
  {
    act_srv_.start();
  }

protected:
  ros::NodeHandle nh_;
  RobotMove& robot_move_;

  actionlib::SimpleActionServer<lwr_peg_in_hole::RobotMoveAction> act_srv_;
  lwr_peg_in_hole::RobotMoveFeedback feedback_;
  lwr_peg_in_hole::RobotMoveResult result_;
  bool robot_moved_;

  void executeCB(const lwr_peg_in_hole::RobotMoveGoalConstPtr& goal)
  {
    robot_moved_ = false;
    ros::Timer mover_thread = nh_.createTimer(ros::Duration(0.001), 
        boost::bind(&RobotMoveActionServer::moveBinCB, this, goal, _1), true);

    ros::Rate r(30);
    while (ros::ok()) {
      if(robot_moved_) {
        ROS_INFO("RobotMove action complete.");
        result_.success = true;
        act_srv_.setSucceeded(result_);
        return;
      }
      if(act_srv_.isPreemptRequested()) {
        ROS_INFO("Preempting robot move");
        mover_thread.stop();
        return;
      }
      ros::spinOnce();
      r.sleep();
    }
  }

  void moveBinCB(const lwr_peg_in_hole::RobotMoveGoalConstPtr& goal, const ros::TimerEvent& te)
  {
    std::vector<double> goal_vals = goal->goal_vals;
    if(goal->cartesian_goal){
      geometry_msgs::Pose cart_pose_goal;
      cart_pose_goal.position.x = goal_vals[0];
      cart_pose_goal.position.y = goal_vals[1];
      cart_pose_goal.position.z = goal_vals[2];
      cart_pose_goal.orientation.x = goal_vals[3];
      cart_pose_goal.orientation.y = goal_vals[4];
      cart_pose_goal.orientation.z = goal_vals[5];
      cart_pose_goal.orientation.w = goal_vals[6];
      robot_move_.moveToCartesianPose(cart_pose_goal);
    }
    else{
      robot_move_.moveToJointPosition(goal_vals);
    }
    robot_moved_ = true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_move_action_server");
  usleep(1000*1000);

  ros::NodeHandle nh;
  RobotMove robot_move;

  RobotMoveActionServer robot_move_as(nh, robot_move);
  ros::spin();
  ros::shutdown();
  return 0;
}