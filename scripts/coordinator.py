#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

def GridCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    pub = rospy.Publisher('coordination', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.2) #hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

