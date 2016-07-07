#! /usr/bin/python
import sys
import yaml
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
      
      #self.object_spawn_ac = actionlib.SimpleActionClient('robot_mover', RobotMoveAction)
      #print "Waiting for action server 'robot_mover' ..."
      #self.object_spawn_ac.wait_for_server()
      #print "Found action server 'robot_mover' !"
      
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


def main():
  rospy.init_node('peg_in_hole_demo1')
  
  demo1 = Demo1()
  demo1.spawnObject("plaque1", 100)
  demo1.spawnObject("plaque2", 100)
  

if __name__ == "__main__":
    main()