#include <lwr_peg_in_hole/robot_move.hpp>
#include <unistd.h>
#include <lwr_peg_in_hole/UpdateSceneService.h>
#include <lwr_peg_in_hole/EstimateHolesService.h>
#include <lwr_peg_in_hole/FindHolePoseService.h>
#include <lwr_peg_in_hole/HoleVisualServoingAction.h>
#include <lwr_peg_in_hole/ScrewdriverAction.h>
#include <std_srvs/Empty.h>
#include <krl_msgs/SetTool.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_a_plat");
  ros::NodeHandle nh("");

  bool sim = false;
  RobotMove robot_move(sim);

  // SET TOOL 10
  krl_msgs::SetToolRequest tool_req;
  krl_msgs::SetToolResponse tool_resp;
  tool_req.tool_number = 10;
  ros::service::call("/lwr_krl_tool/set_tool",tool_req,tool_resp);
  ROS_INFO("Trying to set tool %d", tool_req.tool_number);
  
  // Set up services
  ros::ServiceClient up_client = nh.serviceClient<lwr_peg_in_hole::UpdateSceneService>("update_scene");
  std_srvs::EmptyRequest empty_req;
  std_srvs::EmptyResponse empty_resp;
  lwr_peg_in_hole::UpdateSceneServiceRequest req_update;
  lwr_peg_in_hole::UpdateSceneServiceResponse resp_update;
  req_update.object_name = "plaque2";
  req_update.tag_id = 100;
  while(!up_client.exists())
  {
    ROS_INFO("Waiting for update_scene, estimate_holes and find_hole_pose services...");
    sleep(1.0);
  }
  ROS_INFO("Services ready !");
  
  // Set up action servers
  actionlib::SimpleActionClient<lwr_peg_in_hole::HoleVisualServoingAction> visual_ac("hole_visual_servoing");
  ROS_INFO("Waiting for servers ...");
  visual_ac.waitForServer();
  actionlib::SimpleActionClient<lwr_peg_in_hole::ScrewdriverAction> screwdriver_ac("screwdriver");
  screwdriver_ac.waitForServer();
  ROS_INFO("Servers ready !");
  
  // Wait for ENTER
  std::cout << "\n \n \n Waiting for input ..... Press ENTER ! \n \n \n ";
  std::cin.ignore();

  // Config for specific poses in the demo
  double look_up1_tab[] = {1.64647737821113, -0.06401549522493077, -0.37189732272924836, -1.7298683559749657, -0.010311991928772635, 1.2837615843359318, -1.5951452987645744};
  std::vector<double> look_up1;
  look_up1.assign(look_up1_tab,look_up1_tab+7);

  geometry_msgs::Pose first_fastener_up_lin;
  tf::Quaternion perfect_orientation;
  perfect_orientation.setRPY(-M_PI,0,-(M_PI/2.0 + 20*M_PI/180.0));
  first_fastener_up_lin.orientation.x = perfect_orientation.x();
  first_fastener_up_lin.orientation.y = perfect_orientation.y();
  first_fastener_up_lin.orientation.z = perfect_orientation.z();
  first_fastener_up_lin.orientation.w = perfect_orientation.w();
  first_fastener_up_lin.position.x = -0.46275;
  first_fastener_up_lin.position.y = -0.33885;
  first_fastener_up_lin.position.z = 0.2;

  geometry_msgs::Pose second_fastener_up_lin;
  second_fastener_up_lin.orientation.x = perfect_orientation.x();
  second_fastener_up_lin.orientation.y = perfect_orientation.y();
  second_fastener_up_lin.orientation.z = perfect_orientation.z();
  second_fastener_up_lin.orientation.w = perfect_orientation.w();
  second_fastener_up_lin.position.x = -0.49632;
  second_fastener_up_lin.position.y = -0.39574;
  second_fastener_up_lin.position.z = 0.2;  
  
  
  /*********** DEMO script *****************/
  // Go to start position
  ROS_INFO("Go to start");
  robot_move.moveToStart(100.0);
  
  /////// First fastener ///////
  // Go to first_fastener_up
  ROS_INFO("Go above first fastener");
  robot_move.moveToCartesianPose(first_fastener_up_lin,100.0);
  ROS_INFO("Go down");
  robot_move.moveToHeight(0.08,30.0);
  
  // Get fastener
  ROS_INFO("Go down slowly");
  geometry_msgs::Pose get_fastener;
  get_fastener.orientation.w = 1.0;
  get_fastener.position.z = 0.092;
  robot_move.moveLinRelInTool(get_fastener, 2.0);
  
  ROS_INFO("Taking fastener ...");
  lwr_peg_in_hole::ScrewdriverGoal open_screwdriver_goal;
  open_screwdriver_goal.opening = 1;
  screwdriver_ac.sendGoalAndWait(open_screwdriver_goal);
  usleep(5E5);
  ROS_INFO("Fastener in place !");
  
  // Move up
  ROS_INFO("Go up slowly");
  geometry_msgs::Pose move_up;
  move_up.orientation.w = 1.0;
  move_up.position.z = -0.09;
  robot_move.moveLinRelInTool(move_up, 10.0);
  
  // Move to height 0.2
  ROS_INFO("Go up");
  robot_move.moveToHeight(0.2,100.0);
  
  // Go to lookup position
  ROS_INFO("Go to look_up position");
  robot_move.moveToJointPosition(look_up1, 100.0);
  
  // Wait 1s
  usleep(1E6);
  
  // Update object position in moveit planning scene
  while(!up_client.call(req_update,resp_update) && ros::ok())
  {
      usleep(1E6);
      ROS_INFO("Waiting for object...");
  }
  
  // Read holes poses
  robot_move.loadHolesLocation("plaque2");
  
  // Move to above first hole
  ROS_INFO("Go above first hole");
  robot_move.moveAboveObjectHole("plaque2", 0, 100.0);
  
  // Move A Plat
  ROS_INFO("Go a plat");
  robot_move.moveAPlat(100.0);
  
  // Set bias of ati_sensor
  ros::service::call("/ft_sensor/set_bias",empty_req,empty_resp);
  
  // Move to height 0.12
  ROS_INFO("Go down");
  robot_move.moveToHeight(0.12,30.0);
  
  
  // Visual servoing on the hole
  usleep(2E6);
  ROS_INFO("Running visual servoing...");
  lwr_peg_in_hole::HoleVisualServoingGoal visual_goal;
  visual_goal.save_hole = false;
  visual_ac.sendGoalAndWait(visual_goal);
  ROS_INFO("Visual servoing done !");
    
  // Move In Hole
  ROS_INFO("Going down slowly");
  robot_move.moveToHeight(0.0,1.0,true,10.0);
  
  // Screw fastener
  usleep(5E5);
  ROS_INFO("Putting fastener ...");
  lwr_peg_in_hole::ScrewdriverGoal screwdriver_goal;
  screwdriver_goal.opening = 0;
  screwdriver_ac.sendGoalAndWait(screwdriver_goal);
  ROS_INFO("Fastener in place !");
  
  // Move up
  ROS_INFO("Go up slowly");
  robot_move.moveToHeight(0.12,10.0);
  
  // Move up to 0.2
  ROS_INFO("Go up");
  robot_move.moveToHeight(0.20,100.0);
  
  // Save fastener location
  std::vector<double> above_first_fastener;  
  robot_move.getCurrentJointPosition(above_first_fastener);
  
  
  /////// Second fastener ///////
  // Go to second_fastener_up
  ROS_INFO("Go above second fastener");
  robot_move.moveToCartesianPose(second_fastener_up_lin,100.0);
  ROS_INFO("Go down");
  robot_move.moveToHeight(0.08,30.0);
  
  // Get fastener
  ROS_INFO("Go down slowly");
  robot_move.moveLinRelInTool(get_fastener, 2.0);
  
  ROS_INFO("Taking fastener ...");
  open_screwdriver_goal.opening = 1;
  screwdriver_ac.sendGoalAndWait(open_screwdriver_goal);
  usleep(5E5);
  ROS_INFO("Fastener in place !");
  
  // Move up
  ROS_INFO("Go up slowly");
  robot_move.moveLinRelInTool(move_up, 10.0);
  
  // Move up to 0.2
  ROS_INFO("Go up");
  robot_move.moveToHeight(0.20,100.0);
  
  // Move to above second hole
  ROS_INFO("Go above second hole");
  robot_move.moveAboveObjectHole("plaque2", 1, 100.0);
  
  // Move A Plat
  ROS_INFO("Go a plat");
  robot_move.moveAPlat(100.0);
  
  // Set bias of ati_sensor
  ros::service::call("/ft_sensor/set_bias",empty_req,empty_resp);
  
  // Move to height 0.12
  ROS_INFO("Go down");
  robot_move.moveToHeight(0.12,30.0);
  
  // Visual servoing on the hole
  usleep(2E6);
  ROS_INFO("Running visual servoing...");
  visual_ac.sendGoalAndWait(visual_goal);
  ROS_INFO("Visual servoing done !");
    
  // Move In Hole
  ROS_INFO("Go down slowly");
  robot_move.moveToHeight(0.0,1.0,true,10.0);
  
  // Screw fastener
  usleep(5E5);
  ROS_INFO("Putting fastener ...");
  screwdriver_ac.sendGoalAndWait(screwdriver_goal);
  ROS_INFO("Fastener in place !");
  
  // Move up
  ROS_INFO("Go up slowly");
  robot_move.moveToHeight(0.12,10.0);
  
  // Move to height 0.2
  ROS_INFO("Go up");
  robot_move.moveToHeight(0.2,100.0);
  
  
  // Save fastener location
  std::vector<double> above_second_fastener;  
  robot_move.getCurrentJointPosition(above_second_fastener);
  
  
  /////// Taking back the first fastener /////
  ROS_INFO("Go back to first fastener !");
  robot_move.moveToJointPosition(above_first_fastener, 100.0);
  
  // Move to height 0.16
  ROS_INFO("Go down");
  robot_move.moveToHeight(0.12,30.0);
  
  ROS_INFO("Pick up fastener");
  robot_move.moveToHeight(0.0,1.0,true,40.0);
  
  ROS_INFO("Unscrewing fastener...");
  screwdriver_goal.opening = 1;
  screwdriver_ac.sendGoalAndWait(screwdriver_goal);
  ROS_INFO("Fastener unscrewed !");
  
  ROS_INFO("Go up slowly");
  robot_move.moveToHeight(0.12,10.0);
  ROS_INFO("Go up");
  robot_move.moveToHeight(0.2,100.0);
  
  // Go to first_fastener_up
  ROS_INFO("Go above first fastener hole");
  robot_move.moveToCartesianPose(first_fastener_up_lin,100.0);
  ROS_INFO("Go down");
  robot_move.moveToHeight(0.08,30.0);
  
  // Get fastener
  ROS_INFO("Go down slowly");
  get_fastener.orientation.w = 1.0;
  get_fastener.position.z = 0.092;
  robot_move.moveLinRelInTool(get_fastener, 2.0);
  
  ROS_INFO("Putting fastener ...");
  lwr_peg_in_hole::ScrewdriverGoal close_screwdriver_goal;
  close_screwdriver_goal.opening = 0;
  screwdriver_ac.sendGoalAndWait(close_screwdriver_goal);
  usleep(1E6);
  ROS_INFO("Fastener in place !");
  
  // Move up
  ROS_INFO("Go up slowly");
  move_up.orientation.w = 1.0;
  move_up.position.z = -0.09;
  robot_move.moveLinRelInTool(move_up, 10.0);
  
  ROS_INFO("Go up");
  robot_move.moveToHeight(0.2,100.0);
  
  /////// Taking back the second fastener /////
  ROS_INFO("Go back to 2nd fastener !");
  robot_move.moveToJointPosition(above_second_fastener, 100.0);
  
  // Move to height 0.16
  ROS_INFO("Go down");
  robot_move.moveToHeight(0.12,30.0);
  
  ROS_INFO("Pick up fastener");
  robot_move.moveToHeight(0.0,1.0,true,40.0);
  
  ROS_INFO("Unscrewing fastener...");
  screwdriver_goal.opening = 1;
  screwdriver_ac.sendGoalAndWait(screwdriver_goal);
  ROS_INFO("Fastener unscrewed !");
  
  ROS_INFO("Go up slowly");
  robot_move.moveToHeight(0.12,10.0);
  ROS_INFO("Go up");
  robot_move.moveToHeight(0.2,100.0);
  
  // Go to second_fastener_up
  ROS_INFO("Go above second fastener hole");
  robot_move.moveToCartesianPose(second_fastener_up_lin,100.0);
  ROS_INFO("Go down");
  robot_move.moveToHeight(0.08,30.0);
  
  // Get fastener
  ROS_INFO("Go down slowly");
  get_fastener.orientation.w = 1.0;
  get_fastener.position.z = 0.092;
  robot_move.moveLinRelInTool(get_fastener, 2.0);
  
  ROS_INFO("Putting fastener ...");
  close_screwdriver_goal.opening = 0;
  screwdriver_ac.sendGoalAndWait(close_screwdriver_goal);
  usleep(1E6);
  ROS_INFO("Fastener in place !");
  
  // Move up
  ROS_INFO("Go up slowly");
  move_up.orientation.w = 1.0;
  move_up.position.z = -0.09;
  robot_move.moveLinRelInTool(move_up, 10.0);
  
  ROS_INFO("Go up");
  robot_move.moveToHeight(0.2,100.0);
  
  // Go to start position
  ROS_INFO("Go back to start");
  robot_move.moveToStart(100.0);
  
  ros::shutdown();
  return 0;
}