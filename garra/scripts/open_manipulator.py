#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
#from apriltag_ros.msg import AprilTagDetectionArray
from moveit_commander.conversions import pose_to_list

class openManipulator:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('moveit_routine', anonymous=True)        
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # DEFINE PARAMETERS
        self.group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
        self.group.set_planning_time(5)                      # TIME TO PLANNING

    def go_to_pose_controlled(self, pose_name):
        self.group.set_named_target(pose_name)
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()


    def rotate_joint1_90(self,j):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = j
        joint_goal[1] = j/2
        self.group.go(joints=joint_goal, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

if __name__ == '__main__':
    try:
        manip = openManipulator()
        raw_input("Press Enter to start!")
        print ("moving to down")
        manip.rotate_joint1_90(0)     
        print ("moving to right")
        manip.rotate_joint1_90(-pi/2)
        print ("moving to home")
        manip.go_to_pose_controlled("home")




    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
