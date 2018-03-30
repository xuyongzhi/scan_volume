import rospy,rosbag


rawBagName = 'real_5-11/slow_A1_sand_2017-05-11-15-04-30.bag'

def split(start_min,end_min):
    start_sec = (start_min-4.5)*60
    end_sec = (end_min-4.5)*60

    subBagName_0 = 'real_5-11/' + str(start_min) + '-' + str(end_min) + '.bag'

    t0 = []

    with rosbag.Bag(subBagName_0, 'w') as outbag:
         for topic, msg, t in rosbag.Bag(rawBagName).read_messages(topics='/scan'):
             if t0 == []:
                 t0 = t
             dt = t - t0
             # print "dt = ",(dt.to_sec())
             if dt.to_sec() > end_sec:
                 break
             if dt.to_sec() > start_sec:
                outbag.write(topic, msg, t)
    print "save ",subBagName_0,"  OK"

split(18.2,18.4)