#!/usr/bin/env python

import rospy
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rotate_to_3D import RotateTo3D
import matplotlib.pyplot as plt
import time
import math

Rotate_To_3D = RotateTo3D()
t0 = None
g_scann = -1

def callback(data):
    global t0, g_scann
    g_scann += 1
    if g_scann == 0:
        t0 = time.time()

    scanN, theta = Rotate_To_3D.push(data)
    if scanN > 0 and scanN %50 == 0:
        frequency = g_scann / (time.time()-t0)
        rospy.loginfo(rospy.get_caller_id() + 'scanN = %f, theta = %f, fre=%0.2f',scanN, theta*180.0/math.pi, frequency)

def start_stop( start_stop_str ):
    if start_stop_str.data == 'start':
        rospy.loginfo('send start command')
        Rotate_To_3D.start()
    elif start_stop_str.data == 'stop':
        rospy.loginfo('send stop command')
        Rotate_To_3D.stop()

def listener():
    rospy.init_node('rotate_to_3D',anonymous=True)
    rospy.Subscriber('scan',LaserScan,callback)
    rospy.Subscriber('start_stop_command', String, start_stop)
    rospy.loginfo('rotate_to_3D is ready, waiting for start command\nThe 3d result topic is: pcl_3d')
    rospy.spin()

if __name__ == '__main__':
    listener()
