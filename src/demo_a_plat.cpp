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
  std::cout << "Trying to set tool "<<tool_req.tool_number<<std::endl;;
  
  // Set up services
  ros::ServiceClient up_client = nh.serviceClient<lwr_peg_in_hole::UpdateSceneService>("update_scene");
  ros::ServiceClient estimate_client = nh.serviceClient<lwr_peg_in_hole::EstimateHolesService>("estimate_holes");
  std_srvs::EmptyRequest empty_req;
  std_srvs::EmptyResponse empty_resp;
  lwr_peg_in_hole::UpdateSceneServiceRequest req_update;
  lwr_peg_in_hole::UpdateSceneServiceResponse resp_update;
  req_update.object_name = "plaque2";
  req_update.tag_id = 100;
  lwr_peg_in_hole::EstimateHolesServiceRequest req_estimate;
  lwr_peg_in_hole::EstimateHolesServiceResponse resp_estimate;
  req_estimate.object_name = "plaque2";
  while(!up_client.exists() && !estimate_client.exists())
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
  
  double first_fastener_up_tab[] = {0.9190583825111389, 0.42789730429649353, -0.37253227829933167, -1.7496263980865479, 0.18091551959514618, 0.9975607395172119, -0.9158101677894592};
  std::vector<double> first_fastener_up;
  first_fastener_up.assign(first_fastener_up_tab,first_fastener_up_tab+7);

//   Au dessus du trou 1
//   XYZ = [-0.39155, -0.283, 0.080] RPY=[M_PI,0,-120.0*M_PI/180.0]
//   [0.9190583825111389, 0.42789730429649353, -0.37253227829933167, -1.7496263980865479, 0.18091551959514618, 0.9975607395172119, -0.9158101677894592]
  
//   Dans le trou 1
//   XYZ = [-0.39153, -0.283, -0.010] RPY=[M_PI,0,-120.0*M_PI/180.0]
//   [0.9150278568267822, 0.580691933631897, -0.37253355979919434, -1.812914490699768, 0.2822932004928589, 0.800197184085846, -0.9944129586219788]

  double second_fastener_up_tab[] = {0.8893643021583557, 0.30610182881355286, -0.3718636929988861, -1.9161169528961182, 0.1354074478149414, 0.9433703422546387, -0.943113386631012};
  std::vector<double> second_fastener_up;
  second_fastener_up.assign(second_fastener_up_tab,second_fastener_up_tab+7);
  
//   Au dessus du trou 2
//   XYZ = [-0.35987, -0.23188, 0.080] RPY=[M_PI,0,-120.0*M_PI/180.0]
//   [0.8893643021583557, 0.30610182881355286, -0.3718636929988861, -1.9161169528961182, 0.1354074478149414, 0.9433703422546387, -0.943113386631012]
  
//   Dans le trou 2
//   XYZ = [-0.36041, -0.23096, -0.010] RPY=[M_PI,0,-120.0*M_PI/180.0]
//   [0.8779093027114868, 0.4785161316394806, -0.37185221910476685, -1.983768105506897, 0.25586992502212524, 0.7233907580375671, -1.045378565788269]
  
  
  
  /*********** DEMO script *****************/
  // Go to start position
  robot_move.moveToStart();
  
  // Go to first_fastener_up
  robot_move.moveToJointPosition(first_fastener_up);
//   robot_move.moveToJointPosition(second_fastener_up);
  
  // Get fastener
  geometry_msgs::Pose get_fastener;
  get_fastener.orientation.w = 1.0;
  get_fastener.position.z = 0.092;
  robot_move.moveLinRelInTool(get_fastener, 2.0);
  
//   usleep(1E6);
//   ROS_INFO("Putting fastener ...");
//   lwr_peg_in_hole::ScrewdriverGoal open_screwdriver_goal;
//   open_screwdriver_goal.opening = 1;
//   screwdriver_ac.sendGoalAndWait(open_screwdriver_goal);
//   usleep(1E6);
//   ROS_INFO("Fastener in place !");
  
  // Move up
  geometry_msgs::Pose move_up;
  move_up.orientation.w = 1.0;
  move_up.position.z = -0.09;
  robot_move.moveLinRelInTool(move_up, 10.0);
  
  // Go to lookup position
  robot_move.moveToJointPosition(look_up1);
  
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
//   while(!estimate_client.call(req_estimate,resp_estimate) && ros::ok())
//   {
//       usleep(1E6);
//       ROS_INFO("Getting the holes pose estimate...");
//   }
  
  // Move to above first hole
  robot_move.moveAboveObjectHole("plaque2", 0);
  
  // Move A Plat
  robot_move.moveAPlat(30.0);
  
  // Set bias of ati_sensor
  ros::service::call("/ft_sensor/set_bias",empty_req,empty_resp);
  
  // Move A Plat
  robot_move.moveToHeight(0.12,10.0);
  
  usleep(2E6);
  // Visual servoing on the hole
  lwr_peg_in_hole::HoleVisualServoingGoal visual_goal;
  visual_goal.save_hole = false;
  visual_ac.sendGoalAndWait(visual_goal);
    
  // Move In Hole
  robot_move.moveToHeight(0.0,1.0,true,3.0);
  
  // Screw fastener
  usleep(5E5);
  ROS_INFO("Putting fastener ...");
  lwr_peg_in_hole::ScrewdriverGoal screwdriver_goal;
  screwdriver_goal.opening = 0;
  screwdriver_ac.sendGoalAndWait(screwdriver_goal);
  ROS_INFO("Fastener in place !");
  
  // Move up
  robot_move.moveToHeight(0.16,10.0);
  
  
  /////// Second fastener ///////
  
  // Go to start position
  robot_move.moveToStart();
  
  // Go to first_fastener_up
  robot_move.moveToJointPosition(second_fastener_up);
  
  // Get fastener
  robot_move.moveLinRelInTool(get_fastener, 2.0);
  
//   usleep(1E6);
//   ROS_INFO("Putting fastener ...");
//   lwr_peg_in_hole::ScrewdriverGoal open_screwdriver_goal;
//   open_screwdriver_goal.opening = 1;
//   screwdriver_ac.sendGoalAndWait(open_screwdriver_goal);
//   usleep(1E6);
//   ROS_INFO("Fastener in place !");
  
  // Move up
  robot_move.moveLinRelInTool(move_up, 10.0);
  
  // Go to lookup position
  robot_move.moveToJointPosition(look_up1);
  
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
//   while(!estimate_client.call(req_estimate,resp_estimate) && ros::ok())
//   {
//       usleep(1E6);
//       ROS_INFO("Getting the holes pose estimate...");
//   }
  
  // Move to above first hole
  robot_move.moveAboveObjectHole("plaque2", 1);
  
  // Move A Plat
  robot_move.moveAPlat(30.0);
  
  // Set bias of ati_sensor
  ros::service::call("/ft_sensor/set_bias",empty_req,empty_resp);
  
  // Move A Plat
  robot_move.moveToHeight(0.12,10.0);
  
  usleep(2E6);
  // Visual servoing on the hole
  visual_ac.sendGoalAndWait(visual_goal);
    
  // Move In Hole
  robot_move.moveToHeight(0.0,1.0,true,3.0);
  
  // Screw fastener
  usleep(5E5);
  ROS_INFO("Putting fastener ...");
  screwdriver_ac.sendGoalAndWait(screwdriver_goal);
  ROS_INFO("Fastener in place !");
  
  // Move up
  robot_move.moveToHeight(0.16,10.0);
  
  robot_move.moveToStart();
  
  ros::shutdown();
  return 0;
}