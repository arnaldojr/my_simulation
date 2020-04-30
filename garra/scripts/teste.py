#!/usr/bin/env python

import moveit_commander
import geometry_msgs
import rospy
import sys


def wait_for_state_update(box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0
        is_known = box_name in scene.get_known_object_names()
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
        rospy.sleep(0.1)
        seconds = rospy.get_time()
    return False

if __name__ == '__main__':
    rospy.init_node('attach_object_async', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    
    scene = moveit_commander.PlanningSceneInterface()
    # ROS answers suggested adding a sleep here to allow the PlanningSceneInterface
    # to initialize properly; we have noticed that collision objects aren't
    # always received without it.
    rospy.sleep(2.0)
    
    # create the box we will be using to check for collision detection
    box_name = 'box'
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "link5"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.2
    box_pose.pose.position.x = 0.6
    scene.add_box(box_name, box_pose, size=(0.4, 0.4, 0.4))
    rospy.loginfo(wait_for_state_update(box_name, scene, box_is_known=True))
    
    # create the attached object we will be putting in collision with the box
    box_name = 'tool'
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "link1"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.3
    
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')
    eef_link = group.get_end_effector_link()
    
    grasping_group = 'gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links, pose=box_pose, size=(0.1, 0.1, 0.4))
    rospy.loginfo(wait_for_state_update(box_name, scene, box_is_attached=True, box_is_known=False))