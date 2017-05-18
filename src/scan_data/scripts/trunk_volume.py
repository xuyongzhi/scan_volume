#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Int64
from laser_geometry import LaserProjection
import numpy as np
import matplotlib.pyplot as plt


# each scan is along x axis
# raw z is negative

class TrunkVolumeDetector:
    def __init__(self):
        self.separate_models = False
        self.auto_pub_ref_at_frame = 5

        self.save_raw_scan = False
        self.raw_scan = []
        
        self.height_axis = 0
        self.trunk_length = 0.43
        self.increment = 0.06 # 0.0012
        self.height_offset =  7  # 0.8
        self.height_sign = -1

        self.dif_threshold=  0.3 #  0.06
        self.x_scope = [1,15]
        self.y_scope = [-5,-15]
	
        self.status = 'waiting'
        self.trunk_pcl = []
        self.scanN = 0
        self.pcl_n = 0
        self.volume = -1
        self.ref_frame_LS = []
        self.trunk_pcl_pub = rospy.Publisher('trunk_pcl',PointCloud2,queue_size=10)
        self.scan_difN_pub = rospy.Publisher('scan_dif_n',Int64,queue_size=10)
        self.scan_difStart_pub = rospy.Publisher('scan_dif_start',Int64,queue_size=10)       
        self.scan_difEnd_pub = rospy.Publisher('scan_dif_end',Int64,queue_size=10)       
        self.started = False

        self.fig_dif = plt.figure()
        self.ax_dif = self.fig_dif.add_subplot(111)
        self.received_n = 0
	
                
    def set_ref(self,pcl_LaserScan):
        self.reset()
        self.ref_points_xyz = self.xyz_from_pcl(pcl_LaserScan)
        self.ref_frame_LS = pcl_LaserScan

    def reset(self):
        self.volume = 0
        self.scanN = 0
        self.trunk_pcl = []



    def add_trunk_data(self,pcl_LaserScan,dif_start,dif_end):

        gen_data = pc2.read_points(pcl_LaserScan, field_names=None, skip_nans=True)
	
        trunk_points = []
        if self.trunk_pcl:
            gen_trunk = pc2.read_points(self.trunk_pcl, field_names=None,skip_nans=True)
            for p in gen_trunk:
                trunk_points.append(list(p))

        scan_points = []
        for idx, p in enumerate(gen_data):
            if idx >= dif_start and idx <= dif_end:
                point_i_3d = list(p)
                point_i_3d[2] = self.height_sign * point_i_3d[self.height_axis] + self.height_offset
                point_i_3d[self.height_axis] = self.scanN * self.increment
                scan_points.append(point_i_3d)
                trunk_points.append(point_i_3d)


        if self.save_raw_scan:
            self.raw_scan.append(scan_points)

        self.trunk_pcl = pc2.create_cloud(pcl_LaserScan.header, pcl_LaserScan.fields, trunk_points)

        # show mean of points

        scan_block_mean = np.mean(np.array(scan_points),axis = 0)
        xy_mean = np.array([0,0])
        xy_mean[self.height_axis] = ( scan_block_mean[2] - self.height_offset ) * self.height_sign
        xy_mean[1-self.height_axis] = scan_block_mean[1-self.height_axis]
        # print "xy_mean = ",xy_mean
        # print "scan_points Num = ", len(scan_points)

    def xyz_from_pcl(self,pcl):
        gen = pc2.read_points(pcl, field_names=None, skip_nans=True)
        points = []
        for p in gen:
            xyz = np.array(list(p)[1:4])
            if points == []:
                points = xyz
            else:
                points = np.vstack((points,xyz))
        return points


    def dif_range(self,points_xyz):
        min_N = min(points_xyz.shape[0],self.ref_points_xyz.shape[0])
        dif = points_xyz[0:min_N,self.height_axis] - self.ref_points_xyz[0:min_N,self.height_axis]
        dif = np.fabs(dif)
        threshold = self.dif_threshold
        dif_N = sum([ d > threshold for d in dif ])

        self.scan_difN_pub.publish(dif_N)
        # if self.scanN == 10:
        #     plt.plot(dif)
        #     plt.show()

        if dif_N > 5:
            dif_start = len(dif)
            dif_end = 0
            for i,d in enumerate(dif):
                # IsInTargetScope =  points_xyz[i,0] > self.x_scope[0] and points_xyz[i,0] < self.x_scope[1]  and points_xyz[i,1] > self.y_scope[0] and points_xyz[i,1] < self.y_scope[1]
                # if not IsInTargetScope:
                #     continue

                if dif_start==len(dif) and d > threshold and i+3<len(dif) and  dif[i+1] > threshold and dif[i+3] > threshold:
                    dif_start = i
                    self.scan_difStart_pub.publish(dif_start)
#                if d > threshold and  i+5 < len(dif)  and  sum(dif[i+1:i+5]>threshold)==5 :
# 		        if dif_start==len(dif) and d > threshold and i+3<len(dif) and  dif[i+1] > threshold and dif[i+3] > threshold:
#                     dif_start = i
#                     self.scan_difStart_pub.publish(dif_start)

                if dif_start < len(dif) and i > dif_start and ( d < threshold or (d > threshold and i==len(dif)-1 ) ):
                    dif_end = i
                    self.scan_difEnd_pub.publish(dif_end)
                    if dif_end - dif_start > 3:
                            break
                    else:
                            # rospy.loginfo('short dif_range: dif_start= %d   dif_end= %d   dif_len= %d',dif_start,dif_end,dif_end-dif_start)
                            dif_start = len(dif)
                            dif_end = 0
			    
            # if dif_start < dif_end:
            #     rospy.loginfo('N = %d  dif_start= %d   dif_end= %d   dif_len= %d',self.scanN, dif_start,dif_end,dif_end-dif_start)

    #	    else:
    #		    rospy.loginfo('dif range fail, N = %d  dif_start= %d   dif_end= %d   dif_len= %d',self.scanN, dif_start,dif_end,dif_end-dif_start)

            return True,dif_start,dif_end
        else:
            return False,0,0


    def update_scan_increment(self):
        '''
        do this at the end
	    '''
        self.increment = self.trunk_length / self.scanN
        rospy.loginfo('increment = %f / %d = %f',self.trunk_length,self.scanN,self.increment)


    def push(self,data_LaserScan):
        
        # rospy.loginfo('project data_LaserScan to PointCloud OK')
        pcl_LaserScan = LaserProjection().projectLaser(data_LaserScan)
        points_xyz = self.xyz_from_pcl(pcl_LaserScan)

        # print "scan point N = ",points_xyz.shape[0],"   /  ", pcl_LaserScan.width, "    rangesN = ",len(data_LaserScan.ranges)

        if self.ref_frame_LS:
            IsDif,dif_start,dif_end = self.dif_range(points_xyz)
            IsStart = self.scanN == 0 and IsDif
            if IsDif:
                if self.status == 'waiting':
                        rospy.loginfo('start recording')
                        self.status = 'start'
                else:
                    self.status = 'trunk_scanning'
            else:
                if self.status == 'trunk_scanning':
	                    
                    self.status = 'end'
                    # self.update_scan_increment()
			    
                else:
                    self.status = 'waiting'
            if IsDif:
                self.scanN = self.scanN + 1
                self.add_trunk_data(pcl_LaserScan,dif_start,dif_end)
                self.trunk_pcl_pub.publish(self.trunk_pcl)
        #	rospy.loginfo('pub trunk pcl len = %d',dif_end-dif_start)
            if self.status == 'end':


                if self.save_raw_scan:

                    txt_name = 'pcl_' + str(self.pcl_n) + '.txt'
                    model_txt = open(txt_name,'w')
                    for idx,scan in enumerate( self.raw_scan ):
                        model_txt.write('\n')
                        # Height = []
                        for d in scan:
                            model_txt.write(str(d[1])+'  ')
                            # Height.append(d[1])
                            # if idx == 20 and self.pcl_n==2:
                            #     plt.plot(Height)
                    model_txt.close()

                    rospy.loginfo('save this model: ' + txt_name)

                if self.separate_models:
                    self.pcl_n = self.pcl_n + 1
                    self.reset()
                bag_name = 'pcl_'+str(self.pcl_n)+'.bag' 
                model_bag = rosbag.Bag( bag_name,'w')
                model_bag.write('trunk_pcl',self.trunk_pcl)
                model_bag.close()
                rospy.loginfo('stop recording, save this model: ' + bag_name )


        self.received_n += 1
        if self.received_n == self.auto_pub_ref_at_frame:
            rospy.loginfo('auto set ref at '+str(self.auto_pub_ref_at_frame))
            self.set_ref(pcl_LaserScan)
        return self.volume


    def pcl_volume(self,model_pcl):
        N = model_pcl.size()
        gen_pcl = pc2.read_points(model_pcl, field_names=None, skip_nans=True)
        area_sum = 0
        last_x = NaN
        for idx, p in enumerate(gen_pcl):
            if idx > 0:
                dx = p[0] - last_x
                if dx > 0:
                    area = dx * (p[2]+0.7)
                    area_sum = area_sum + area
        dy = self.trunk_length / (N-1)
        volume = dy * area_sum
        rospy.loginfo('volume = %f',volume)
        return volume

    def volume_from_bag(self,model_bag_file):
        model_bag = rosbag.Bag(model_bag_file)
        msg_gen = model_bag.read_messages(topics='trunk_pcl')
        for topic,msg,t in msg_gen:
            self. pcl_volume(msg)


if __name__ == '__main__':
    print 'in main'
    
    TVD = TrunkVolumeDetector()
    TVD.volume_from_bag('model_result_new/empty.bag')





