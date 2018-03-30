#!/usr/bin/env python

import rospy
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from trunk_volume import TrunkVolumeDetector
import matplotlib.pyplot as plt

TV_detector = TrunkVolumeDetector()

cur_frame = []

def callback(data):
    global cur_frame

    volume = TV_detector.push(data)

    cur_frame = data
    # rospy.loginfo(rospy.get_caller_id() + ' volume = %f',volume)


def record_ref(ref_str):
    global cur_frame

    if ref_str.data == 'ref' and cur_frame:
            TV_detector.set_ref(cur_frame)
            rospy.loginfo('record the reference frame')
    else:
            if not cur_frame:
                    rospy.loginfo(' set ref failed, cur_frame=[]')
            else:
                    rospy.loginfo('set ref failed,str %s wrong',ref_str)




def listener():

    rospy.init_node('trunk_scanner',anonymous=True)
    rospy.Subscriber('scan',LaserScan,callback)
    rospy.Subscriber('ref_command',String,record_ref)
    rospy.loginfo('trunk_scaner started, waiting for ref command')
    rospy.spin()

    # plt.show()

if __name__ == '__main__':
    listener()
