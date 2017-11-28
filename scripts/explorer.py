#!/usr/bin/env python

import rospy
import cv2
import tf
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


def GridCallback(data):
    position, quat = tf.lookupTransform(, "world", time) 
    local_map = OccupancyGridToMat(data)
    home = cv2.imread("../world/home.pgm")

    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(local_map, None);
    kp2, des2 = orb.detectAndCompute(home, None);
    img2 = np.zeros((0,0), np.uint8)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors.
    matches = bf.match(des1,des2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    # Draw first 10 matches.
    img3 = np.zeros((0,0), np.uint8)
    img3 = cv2.drawMatches(local_map,kp1,home,kp2,matches[:10], img3, flags=2)

    cv2.imshow("map", local_map)
    cv2.imshow("home", home)
    cv2.waitKey(0)
    plt.imshow(img3),
    plt.show()

def OccupancyGridToMat(Occ):
    width = Occ.info.width
    height = Occ.info.height
    rospy.get_published_topics();
    local_map = np.zeros((width, height, 1), dtype="uint8")
    for i in range(0, len(Occ.data)):
        if(Occ.data[i] == -1):
            value = 0
        else:
            value = Occ.data[i]
        local_map[i%width, i/height,] = 255 - value*255.0
    return local_map

def talker():

    prefix = rospy.get_param("prefix")

    pub = rospy.Publisher('coordination', String, queue_size=10)

    rospy.Subscriber(prefix + "/map", OccupancyGrid, GridCallback)
    rospy.Subscriber(prefix + "/merged_map", OccupancyGrid, GridCallback)

    rospy.init_node('talker', anonymous=False)
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

