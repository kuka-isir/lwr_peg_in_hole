#! /usr/bin/python
import sys
import yaml
import time
import numpy as np
import actionlib
import rospy

from lwr_peg_in_hole.msg import ObjectSpawnAction, ObjectSpawnGoal, RobotMoveAction, RobotMoveGoal, VisualServoingAction, VisualServoingGoal
from std_msgs.msg import Int32, String, Bool, Int8, Int8MultiArray

class Demo1(object):
  def __init__(self):
    
    #self.occupied_sub = rospy.Subscriber('/human/workspace/occupied', Bool, self.occupied_cb)
    #self.scanned_parts_pub = rospy.Publisher('/display/scanned_parts', Int8, latch=True)

    self.object_spawn_ac = actionlib.SimpleActionClient('object_spawner', ObjectSpawnAction)
    print "Waiting for action server 'object_spawner' ..."
    self.object_spawn_ac.wait_for_server()
    print "Found action server 'object_spawner' !\n"
    
    self.robot_move_ac = actionlib.SimpleActionClient('robot_mover', RobotMoveAction)
    print "Waiting for action server 'robot_mover' ..."
    self.robot_move_ac.wait_for_server()
    print "Found action server 'robot_mover' !"
    
    #self.object_spawn_ac = actionlib.SimpleActionClient('visual_servoer', VisualServoingAction)
    #print "Waiting for action server 'visual_servoer' ..."
    #self.object_spawn_ac.wait_for_server()
    #print "Found action server 'visual_servoer' !"
        
  def spawnObject(self, object_name, tag_id):
    
    spawn_object_goal = ObjectSpawnGoal()
    spawn_object_goal.object_name = object_name
    spawn_object_goal.tag_id = tag_id
    
    print "Sending ObjectSpawn goal:\n", spawn_object_goal, "\n"
    result = self.object_spawn_ac.send_goal_and_wait(spawn_object_goal)
    return result == actionlib.GoalStatus.SUCCEEDED
  
  def moveToCartesianGoal(self, cart_pose):
    
    robot_move_goal = RobotMoveGoal()
    robot_move_goal.cartesian_goal = True
    robot_move_goal.goal_vals = cart_pose
    
    print "Sending RobotMove goal:\n", robot_move_goal, "\n"
    result = self.robot_move_ac.send_goal_and_wait(robot_move_goal)
    return result == actionlib.GoalStatus.SUCCEEDED
  
  def moveToJointPosition(self, joint_position):
    
    robot_move_goal = RobotMoveGoal()
    robot_move_goal.cartesian_goal = False
    robot_move_goal.goal_vals = joint_position
    
    print "Sending RobotMove goal:\n", robot_move_goal, "\n"
    #result = self.robot_move_ac.send_goal_and_wait(robot_move_goal)
    self.robot_move_ac.send_goal(robot_move_goal)
    time.sleep(0.1)
    self.robot_move_ac.cancel_goal()
    self.robot_move_ac.stop_tracking_goal()
    
    #return result == actionlib.GoalStatus.SUCCEEDED
    return



def main():
  rospy.init_node('peg_in_hole_demo1')
  
  demo1 = Demo1()

  demo1.spawnObject("plaque1", 100)
  demo1.spawnObject("plaque2", 100)
  
  start_pose = [1.6002605785960622, -0.0449239937131054, -0.333517572028704, -1.707667848031405, -4.734100141323694e-05, 1.3883824216456677, -0.00020880873042017356]
  demo1.moveToJointPosition(start_pose)
  
  pose_cart1 = [0.369, 0.078, 0.410, 0.993, 0.110, 0.036, -0.011]
  demo1.moveToCartesianGoal(pose_cart1)
  
  print "Result :\n" ,demo1.robot_move_ac.get_result().success
  

if __name__ == "__main__":
    main()