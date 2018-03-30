#!/usr/bin/env python

import rospy
import rosbag

trunk_pcl_bag = rosbag.Bag('trunk_pcl.bag')
for topic, msg, t in trunk_pcl_bag.read_messages(topics=['trunk_pcl']):
	print msg
trunk_pcl_bag.close()
