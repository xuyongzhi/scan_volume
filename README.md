# scan_volume
scan moving truck with 2d Lidar LMS111, extract 3d model of empty truck and filled truck to caclate volume of sand/soil/clay

## Install
	http://wiki.ros.org/LMS1xx
	At root path, catkin_make

## Set env:
	cd devel,  source setup.bash

## Connect
	Physical: Connect with ethernet cable.
	Software: -->“Edit Connections” -->Add Ethernet connection name as ”LMS111” -->IPV4 -->Create a new IP:192.168.0.4,Netmask：192.168.255.255，Gateway：192.168.0.1
	rosrun lms1xx LMS1xx_node _host:=192.168.0.1
	rosrun rviz rviz
	In rviz: Add --> by topic --> LaserScan --> FixdFrame: change map to laser

real volume = 3.93 L

z_offset = 0.684
sand1_volume = 8.975043 - 5.861451 = 3.1136	20.77%3
sand2_volume = 9.274320 - 5.861451 = 3.4129	13.158%


z_offset = 0.884
sand1_volume = 29.166270 - 25.996145 = 3.17012  19.33%
sand2_volume = 29.490175 - 25.996145 = 3.49403	11.093%

raw_pcl:
![Alt text](/data/simu-4-28-data/raw_pcl/empty_whole-d-intensityColor.png)
![Alt text](/data/simu-4-28-data/raw_pcl/trucksand2a.png)
![Alt text](/data/simu-4-28-data/optimized_pcl/empty_optimized_b.png)
![Alt text](/data/simu-4-28-data/optimized_pcl/sand1_od_d.png)
![Alt text](/data/5-11-real-truck-data/A30e_empty-11.2-11.5-a.png)
![Alt text](/data/5-11-real-truck-data/A40D_clay-zColor-1.png)

