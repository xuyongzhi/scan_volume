rostopic pub -1 'ref_command' std_msgs/String 'ref'  &
rosbag play -r 2 9.7-10.bag
