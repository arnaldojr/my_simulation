#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

tf_buffer = tf2_ros.Buffer()


markers_presentes = [11,12,13,21,22,23]

marker_frames = ["ar_marker_{}".format(i) for i in markers_presentes]

def check_transform(frame1, frame2):
    #print("Verificando se possível transformar de ", frame1, " para ", frame2)
    possible = tf_buffer.can_transform(frame1, frame2, rospy.Time(0))
    if possible:
        # print(" de ", frame1, " para ", frame2,  ": ", possible)
        trans = tf_buffer.lookup_transform(a, b, rospy.Time(0))
        # print(trans)
        return trans 
    else:
        return 0


if __name__ == "__main__":
    rospy.init_node("transform_checker")

    tfl = tf2_ros.TransformListener(tf_buffer)

    base = ["base_footprint", "odom", "base_link", "camera_link", "camera_rgb_optical_frame"]
    pontos= ["joint1", "joint2", "joint3", "joint4", "end_effector_link", "gripper", "gripper_sub", "camera_link", "camera_rgb_optical_frame", "base_footprint", "odom"]
    pontos.extend(marker_frames)
    
    pares = [(x,y) for x in pontos for y in base if x!=y]

    print("Pares possíveis: ", pares)

    while not rospy.is_shutdown():
        possible = []
        impossible = []
        try:
            for p in pares:
                a,b = p
                trans = check_transform(a,b)
                if trans!=0: 
                    """Guarda em uma lista uma
                       transformação que é possível """ 
                    possible.append(p)
                else:
                    impossible.append(p)
                trans = check_transform(b,a)
                if trans!=0:
                    possible.append((b,a))

            print("\rPares possíveis: \n")
            print("\r", possible," \r")
            print("\rPares impossíveis: \n")
            print("\r", impossible," \r")
            rospy.sleep(4)

        except rospy.ROSInterruptException as e:
            print(e)
        except KeyboardInterrupt:
            pass
