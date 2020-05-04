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
import time
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


    def rotate_joint_x(self,j):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = j
        self.group.go(joints=joint_goal, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()


    def rotate_joint_y(self,j):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[1] = j
        self.group.go(joints=joint_goal, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def rotate_joint_z(self,j):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[2] = j
        self.group.go(joints=joint_goal, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def rotate_joint_gripper(self,j):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[3] = j
        self.group.go(joints=joint_goal, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()


class openManipulatorGripper:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('moveit_routine', anonymous=True)        
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "gripper"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # DEFINE PARAMETERS
        self.group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
        self.group.set_planning_time(5)                      # TIME TO PLANNING

    def gripper(self, pose_name):
        print ("controlando a garra")
        self.group.set_named_target(pose_name)
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()




if __name__ == '__main__':
    try:
        manip = openManipulator()
        gripper = openManipulatorGripper()
        print ("moving to home")
        manip.go_to_pose_controlled("home")
        print ("Pegando o Creeper")
        manip.rotate_joint_x(0)        
        manip.rotate_joint_y(0) 
        manip.rotate_joint_z(0) 
        manip.rotate_joint_gripper(0) 
        print ("levando o Creeper")
        gripper.gripper("close")
        time.sleep(5)
        manip.rotate_joint_x(1)        
        manip.rotate_joint_gripper(0) 
        gripper.gripper("open")
        time.sleep(5)
        print ("Done")       
  

        #print ("moving to right")
        #manip.rotate_joint1_90(-pi/2)






    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
