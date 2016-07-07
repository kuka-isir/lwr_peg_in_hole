#include <actionlib/server/simple_action_server.h>
#include <lwr_peg_in_hole/ObjectSpawnAction.h>
#include <lwr_peg_in_hole/object_spawn.hpp>

class ObjectSpawnActionServer
{
public:
  ObjectSpawnActionServer(ros::NodeHandle& nh, ObjectSpawn& object_spawn) :
    nh_(nh), object_spawn_(object_spawn),
    act_srv_(nh, "object_spawner", boost::bind(&ObjectSpawnActionServer::executeCB, this, _1), false)
  {
    act_srv_.start();
  }

protected:
  ros::NodeHandle nh_;
  ObjectSpawn& object_spawn_;

  actionlib::SimpleActionServer<lwr_peg_in_hole::ObjectSpawnAction> act_srv_;
  lwr_peg_in_hole::ObjectSpawnFeedback feedback_;
  lwr_peg_in_hole::ObjectSpawnResult result_;
  bool object_spawned_;

  void executeCB(const lwr_peg_in_hole::ObjectSpawnGoalConstPtr& goal)
  {
    object_spawned_ = false;
    ros::Timer spawner_thread = nh_.createTimer(ros::Duration(0.001), 
        boost::bind(&ObjectSpawnActionServer::moveBinCB, this, goal, _1), true);

    ros::Rate r(30);
    while (ros::ok()) {
      if(object_spawned_) {
        ROS_INFO("ObjectSpawn action complete.");
        result_.success = true;
        act_srv_.setSucceeded(result_);
        return;
      }
      if(act_srv_.isPreemptRequested()) {
        ROS_INFO("Preempting object spawn");
        spawner_thread.stop();
        return;
      }
      ros::spinOnce();
      r.sleep();
    }
  }

  void moveBinCB(const lwr_peg_in_hole::ObjectSpawnGoalConstPtr& goal, const ros::TimerEvent& te)
  {
    std::string object_name = goal->object_name;
    int tag_id = goal->tag_id;
    object_spawn_.updateObjectInScene(object_name, tag_id);
    object_spawned_ = true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_spawn_action_server");
  usleep(1000*1000);

  ros::NodeHandle nh;
  ObjectSpawn object_spawn;

  ObjectSpawnActionServer object_spawn_as(nh, object_spawn);
  ros::spin();
  ros::shutdown();
  return 0;
}