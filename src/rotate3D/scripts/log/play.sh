rosbag play -r 1 ./lms111_360d_2018-03-31-16-01-42.bag &
rostopic pub -1 'start_stop_command' std_msgs/String 'start'
