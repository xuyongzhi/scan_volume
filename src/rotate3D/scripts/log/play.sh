rosbag play -r 1 lms111_180.bag &
rostopic pub -1 'start_stop_command' std_msgs/String 'start'
