rosbag play -r 1 ./lms111_180d_2018-03-31-16-40-14.bag &
rostopic pub -1 'start_stop_command' std_msgs/String 'start'
