#!/usr/bin/env python

import rospy, rosbag
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt

if __name__ == '__main__':
    bag_name = 'scan_ref_bag/empty_trunk_pcl.bag'
    bag  = rosbag.Bag(bag_name)

    for topic,msg,t in bag:
        print topic
        gen = pc2.read_points(msg, field_names=None,skip_nans=True)
        X = []
        Y = []
        Z = []
        for i,p in enumerate(gen):
            d = list(p)
            X.append(d[0])
            Y.append(d[1])
            Z.append(d[2])

        plt.plot(X,label='X')
        plt.plot(Y,label='Y')
        plt.plot(Z,label='Z')
        plt.legend(loc='upper left')

    plt.show()