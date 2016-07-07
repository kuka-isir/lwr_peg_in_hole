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
      print "Found action server 'object_spawner' !"
      
      self.object_spawn_ac = actionlib.SimpleActionClient('robot_mover', RobotMoveAction)
      print "Waiting for action server 'robot_mover' ..."
      self.object_spawn_ac.wait_for_server()
      print "Found action server 'robot_mover' !"
      
      self.object_spawn_ac = actionlib.SimpleActionClient('visual_servoer', VisualServoingAction)
      print "Waiting for action server 'visual_servoer' ..."
      self.object_spawn_ac.wait_for_server()
      print "Found action server 'visual_servoer' !"

    def run(self):
      while not rospy.is_shutdown():
        
        if True:
          rospy.sleep(3.0)
        else:
          print "Press enter to continue"
          sys.stdin.readline()

def main():
  rospy.init_node('peg_in_hole_demo1')
  
  demo1 = Demo1()
  rospy.sleep(2.0)
  demo1.run()

if __name__ == "__main__":
    main()