cmake_minimum_required(VERSION 2.8.3)
project(lwr_peg_in_hole)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_visualization
  roscpp
  rospy
  std_msgs
  krl_msgs
#  descartes_moveit
#  descartes_trajectory
#  descartes_planner
#  descartes_core
  actionlib
  actionlib_msgs
  control_msgs
  cv_bridge
  image_geometry
  image_transport
  tf
  tf_conversions
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UpdateSceneService.srv
  EstimateHolesService.srv
  FindHolePoseService.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ObjectSpawn.action
  RobotMove.action
  HoleVisualServoing.action
  Screwdriver.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  moveit_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lwr_peg_in_hole
#  CATKIN_DEPENDS geometry_msgs moveit_planners_ompl moveit_ros_move_group moveit_ros_visualization roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(object_spawn src/object_spawn.cpp)
add_library(robot_move src/robot_move.cpp)
add_library(hole_visual_servoing src/hole_visual_servoing.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(lwr_peg_in_hole ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(spawn_holes src/spawn_holes.cpp)
#add_executable(cart_path src/cart_path.cpp)
add_executable(update_scene_service src/update_scene_service.cpp)
add_executable(estimate_holes_service src/estimate_holes_service.cpp)
add_executable(object_spawn_action_server src/object_spawn_action_server.cpp)
add_executable(robot_move_action_server src/robot_move_action_server.cpp)
add_executable(screwdriver_action_server src/screwdriver_action_server.cpp)
add_executable(hole_visual_servoing_action_server src/hole_visual_servoing_action_server.cpp)
add_executable(hole_visual_servoing_node src/hole_visual_servoing_node.cpp)
add_executable(robot_move_node src/robot_move_node.cpp)
add_executable(demo1 src/demo1.cpp)
add_executable(demo0 src/demo0.cpp)
add_executable(demo_a_plat src/demo_a_plat.cpp)
add_executable(demo_lin src/demo_lin.cpp)
add_executable(real_demo_test0 src/real_demo_test0.cpp)
add_executable(real_demo_visual_servo_2d src/real_demo_visual_servo_2d.cpp)
add_executable(ellipse_detection src/ellipse_detection.cpp)
add_executable(find_hole_pose_service src/find_hole_pose_service.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(spawn_holes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(cart_path ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(update_scene_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(estimate_holes_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(object_spawn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(object_spawn_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_move_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(screwdriver_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hole_visual_servoing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hole_visual_servoing_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_move_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo0 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(real_demo_test0 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ellipse_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(find_hole_pose_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(spawn_holes ${catkin_LIBRARIES})
#target_link_libraries(cart_path ${catkin_LIBRARIES})
target_link_libraries(update_scene_service ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(estimate_holes_service ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(object_spawn ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(object_spawn_action_server ${catkin_LIBRARIES} object_spawn)
target_link_libraries(robot_move ${catkin_LIBRARIES})
target_link_libraries(robot_move_action_server ${catkin_LIBRARIES} robot_move yaml-cpp)
target_link_libraries(hole_visual_servoing ${catkin_LIBRARIES} )
target_link_libraries(hole_visual_servoing_action_server ${catkin_LIBRARIES} hole_visual_servoing robot_move yaml-cpp)
target_link_libraries(hole_visual_servoing_node ${catkin_LIBRARIES} hole_visual_servoing)
target_link_libraries(robot_move_node ${catkin_LIBRARIES} robot_move yaml-cpp)
target_link_libraries(screwdriver_action_server ${catkin_LIBRARIES})
target_link_libraries(demo1 ${catkin_LIBRARIES} robot_move yaml-cpp)
target_link_libraries(demo0 ${catkin_LIBRARIES} robot_move yaml-cpp)
target_link_libraries(demo_a_plat ${catkin_LIBRARIES} robot_move hole_visual_servoing yaml-cpp)
target_link_libraries(demo_lin ${catkin_LIBRARIES} robot_move yaml-cpp)
target_link_libraries(real_demo_test0 ${catkin_LIBRARIES} robot_move yaml-cpp)
target_link_libraries(real_demo_visual_servo_2d ${catkin_LIBRARIES} robot_move yaml-cpp)
target_link_libraries(ellipse_detection ${catkin_LIBRARIES} )
target_link_libraries(find_hole_pose_service ${catkin_LIBRARIES} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lwr_peg_in_hole lwr_peg_in_hole_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lwr_peg_in_hole.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
