#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


def talker():
    rospy.get_published_topics();

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

