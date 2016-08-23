#include <lwr_peg_in_hole/robot_move.hpp>
#include <unistd.h>
#include <lwr_peg_in_hole/UpdateSceneService.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo1");
  ros::NodeHandle nh("");

  ros::ServiceClient up_client = nh.serviceClient<lwr_peg_in_hole::UpdateSceneService>("update_scene");
  while(!up_client.exists())
  {
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }
  lwr_peg_in_hole::UpdateSceneServiceRequest req;
  lwr_peg_in_hole::UpdateSceneServiceResponse resp;
  req.object_name = "plaque2";
  req.tag_id = 100;


  bool sim = false;
  RobotMove robot_move(sim);

  std::cout << "\n \n \n Waiting for input ..... Press ENTER ! \n \n \n ";
  std::cin.ignore();

  while(!robot_move.moveToStart() && ros::ok())
  {
    usleep(1E6);
    up_client.call(req,resp);
  }

  while(!up_client.call(req,resp) && ros::ok())
  {
    usleep(1E6);
    ROS_INFO("Waiting for object");
  }

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

  robot_move.loadHolesLocation("plaque2");

  while(ros::ok()){
    while(!robot_move.moveToStart() && ros::ok())
    {
      usleep(1E6);
      up_client.call(req,resp);
    }

    up_client.call(req,resp);

    while(!robot_move.moveToJointPosition(first_fastener) && ros::ok()){usleep(1E6);}

    usleep(1E6);

    while(!robot_move.moveToStart() && ros::ok())
    {
      usleep(1E6);
      up_client.call(req,resp);
    }

    while(!robot_move.moveAboveObjectHole("plaque2", 0) && ros::ok()){
      usleep(1E6);
    }

    while(!robot_move.moveToStart() && ros::ok())
    {
      usleep(1E6);
      up_client.call(req,resp);
    }

    while(!robot_move.moveToJointPosition(second_fastener) && ros::ok()){usleep(1E6);}

    usleep(1E6);

    while(!robot_move.moveToStart() && ros::ok())
    {
      usleep(1E6);
      up_client.call(req,resp);
    }

    while(!robot_move.moveAboveObjectHole("plaque2", 1) && ros::ok()){usleep(1E6);};
  }

  ros::shutdown();
  return 0;
}
