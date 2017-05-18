import rospy,rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt

truck_length = 0.43   # 0.41(!!!)  0.43

def optimize_pcl( model_pcl,optimizd_bag_name):
    fig = plt.figure()

    
    N = model_pcl.header.seq
    gen_pcl = pc2.read_points(model_pcl, field_names=None, skip_nans=True)

    last_x = []
    last_y = []
    scan_points = np.array([])
    scan_points_N = []
    new_scan_points_N = []


    scan_Num = 0

    cut_x_start = 0
    cut_x_end = 0
    cut_y_idx_start = []
    cut_y_idx_end = []

    all_raw_scans = []

    for idx, p in enumerate(gen_pcl):

        pp = np.array(list(p))
        pp[2] = pp[2] + 0.684   # 0.684
        # print pp

        if idx > 0: 
            dx = p[0] - last_x
            dy = p[1] - last_y
            # if dx > -0.01:
            if dy < 0.000001:
                if scan_points.size == 0:
                    scan_points = pp
                else:
                    scan_points = np.vstack((scan_points, pp))
            else:
                all_raw_scans.append(scan_points)
                # delete the useless points on both sides
                idx_start = 0
                idx_end = scan_points.shape[0]-1
                for j,s in enumerate( scan_points ):
                    if j>0 and j<10:
                        ds_left = s[2] - scan_points[j-1][2]
                        if ds_left > 0.015:
                            idx_start = j
                        ds_right = scan_points[-j-1][2] - scan_points[-j][2]
                        if ds_right > 0.015:
                            idx_end = -j-1
                        # print "j = ",j, "   ds_left = ",ds_left, "   ds_right = ",ds_right
                # print "idx_start = ",idx_start, "   idx_end = ",idx_end
                new_scan_points = scan_points[idx_start:idx_end,:]
                new_scan_points_n = new_scan_points.shape[0]
                new_scan_points_N.extend([new_scan_points_n])
                scan_points_N.extend([scan_points.shape[0]])
                # print "scan_points_n = ",scan_points_n, "dx = ",dx

                if new_scan_points.shape[0] > 5:
                    if np.amin(new_scan_points,axis=0)[0] < cut_x_start:
                        cut_x_start = np.amin(new_scan_points,axis=0)[0]
                    if np.amax(new_scan_points,axis=0)[0] > cut_x_end:
                        cut_x_end = np.amax(new_scan_points,axis=0)[0]

                scan_Num  = scan_Num+1
                # if scan_Num > N/4:
                #     plt.subplot(211)
                #     plt.plot(scan_points[:,0],scan_points[:,2],'*')
                #     plt.subplot(212)
                #     plt.plot(new_scan_points[:, 0], new_scan_points[:, 2], '*')
                #     plt.show()

                scan_points = pp


        else:
            scan_points = pp
        last_x = p[0]
        last_y = p[1]

    # print "cut_x_start = ",cut_x_start, "  cut_x_end=",cut_x_end
    scan_points_N_max = max(scan_points_N)
    for i,n in enumerate(scan_points_N):
        if n > scan_points_N_max - 3:
            cut_y_idx_start = i
            break
    for i, n in reversed(list(enumerate(scan_points_N))):
        if n > scan_points_N_max - 3:
            cut_y_idx_end = i
            break
    # print "cut_y_idx_start = ",cut_y_idx_start, "    cut_y_idx_end=",cut_y_idx_end

    # optimize all_raw_points
    all_optimized_scans0 = all_raw_scans[cut_y_idx_start:cut_y_idx_end]
    all_optimized_points = []
    all_optimized_scans = []
    optimized_scan_points_N = []

    valid_scan_N = len(all_optimized_scans0)
    dy_mean = truck_length / (valid_scan_N-1)

    for s_idx,scan in enumerate( all_optimized_scans0 ):
        sn = 0
        scan_optimized = []
        for p in scan:
            p[1] = dy_mean * s_idx
            if p[0] > cut_x_start and p[0] < cut_x_end:
                scan_optimized.extend([p])
                sn = sn+1
        all_optimized_points.extend(scan_optimized)
        all_optimized_scans.append(scan_optimized)
        optimized_scan_points_N.extend([sn])


    fig_scan_n = plt.figure
    plt.subplot(111)
    # plt.plot(new_scan_points_N,'r')
    plt.plot(scan_points_N,'g')
    # plt.plot(optimized_scan_points_N,'b')
    plt.show()


    print( "   valid_scan_N = ",valid_scan_N, "     scan_N_total=",len(scan_points_N))


    new_trunk_pcl = pc2.create_cloud(model_pcl.header, model_pcl.fields, all_optimized_points)
    new_trunk_bag = rosbag.Bag(optimizd_bag_name+'.bag','w')
    new_trunk_bag.write('truck_pcl',new_trunk_pcl)
    new_trunk_bag.close()
    print "write ",optimizd_bag_name

    write_scan_txt(all_optimized_scans,optimizd_bag_name+'.txt')

    return all_optimized_points,valid_scan_N

def write_scan_txt(all_optimized_scans,txt_file_name):
    file = open( txt_file_name,'w' )
    file.write('     x    ' + '    ' + '     y     ' + '    ' + '     z     ' + '\n')
    for s in all_optimized_scans:
        for p in s:
            file.write( str(p[0]) + '    ' + str(p[1]) + '    ' + str(p[2]) + '\n' )
        file.write('\n')
    file.close()
    print "save: ", txt_file_name

def pcl_volume( optimized_points,valid_scan_N):
    area_sum = 0
    last_y = []

    minZ = 100
    for p in optimized_points:
        if p[2] < minZ:
            minZ = p[2]
    # print "minZ = ",minZ

    scan_points = np.array([])
    for idx, p in enumerate(optimized_points):
        if idx > 0:
            dy = p[1] - last_y
            if dy > 0:
                scan_z_mean = np.mean(scan_points,axis=0)[2]
                min_x = np.amin(scan_points,axis=0)[0]
                max_x = np.amax(scan_points, axis=0)[0]
                area = (max_x-min_x) * scan_z_mean
                area_sum = area_sum + area

                scan_points = p
            else:
                scan_points = np.vstack((scan_points,p))
        else:
            scan_points = p
        last_y = p[1]
    dy = truck_length / (valid_scan_N - 1)
    print "dy = ",dy
    volume = dy * area_sum * 1000
    print('volume = %f', volume)


    return volume,optimized_points




def volume_from_bag( model_bag_file,optimizd_bag_name):
    model_bag = rosbag.Bag(model_bag_file)
    msg_gen = model_bag.read_messages(topics='trunk_pcl')
    for topic, msg, t in msg_gen:
        optimized_points,valid_scan_N = optimize_pcl(msg,optimizd_bag_name)
        return  pcl_volume(optimized_points,valid_scan_N)



def calculate_sand_volume():
    sand_n = 1
    path = '4-28-lab_pcl_result'

    volume0, optimized_points_0 = volume_from_bag(path+'/empty.bag', path+'/empty_optimized')
    volume1, optimized_points_1 = volume_from_bag(path+'/sand' + str(sand_n) + '.bag',
                                                  path+'/sand' + str(sand_n) + '_optimized')
    sand_v = volume1 - volume0
    error = (sand_v - 3.93) / 3.93
    print "sand volume = ", volume1 - volume0, "  ", error

    empty_sand_points = optimized_points_0
    empty_sand_points.extend(optimized_points_1)

    empty_model_bag = rosbag.Bag(path+'/empty.bag')
    msg_gen = empty_model_bag.read_messages(topics='trunk_pcl')
    for topic, msg, t in msg_gen:
        empty_sand_pcl = pc2.create_cloud(msg.header, msg.fields, empty_sand_points)
        empty_sand_Bag = rosbag.Bag(path+'/empty_sand' + str(sand_n) + '.bag', 'w')
        empty_sand_Bag.write('truck_pcl', empty_sand_pcl)
        empty_sand_Bag.close()
        print
        "write: "+path+"/empty_sand" + str(sand_n) + ".bag"

def show_pcl_info():
    volume0, optimized_points_0 = volume_from_bag('5-11-pcl_result/12.7-15-whole.bag', '5-11-pcl_result/12.7-15-whole-optimized.bag')

if __name__ == '__main__':
    print 'in main'
    calculate_sand_volume()

