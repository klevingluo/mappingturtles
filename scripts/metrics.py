#!/usr/bin/env python

import tf
import rospy
import numpy as np
import math
import json
import cv2
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

class Metrics:
    """ a class for measuring the exploration progress """

    def __init__(self):
        self.data = []
        self.interval = 10
        self.timer = 0
        self.last_logged = 0
        self.travelled = 0
        self.rotated = 0
        self.mapped_area = 0

    def GridCallback(self, data):
        mapped_area = 0
        for i in range(0, len(data.data)):
            if(data.data[i] != -1 and data.data[i] < 50):
                mapped_area += 1
        self.mapped_area = mapped_area

        # local_map = self.OccupancyGridToMat(data)
        # home = cv2.imread("../world/home.pgm", 0)
    
        # listener = tf.TransformListener()
        # rospy.sleep(0.5)
        # position, quat = listener.lookupTransform("world", self.robotname + "/map", rospy.Time(0)) 
        # euler = tf.transformations.euler_from_quaternion(quat)
    
        # rows,cols = local_map.shape
        # # rotation is given by the rotation parameter in the world
        # res = data.info.resolution
        # center = [
        #         [int(-data.info.origin.position.x/res)], 
        #         [int(-data.info.origin.position.y/res)], 
        #         [0]]
        # M = cv2.getRotationMatrix2D((center[0][0],center[1][0]),0,1)
        # local_map = cv2.warpAffine(local_map,M,(cols,rows))
        # # 20 is the resolution, 24 is the size of the map
        # local_map = cv2.resize(
        #         local_map,
        #         (int(local_map.shape[0]*res*home.shape[0]/24), 
        #          int(local_map.shape[1]*res*home.shape[1]/24)))
    
    
        # center = [[-data.info.origin.position.x/res], [-data.info.origin.position.y/res], [0]]
        # # transforming the center with the matrix
        # center = np.dot(M,center)
    
        # centerx = -data.info.origin.position.x*res*24
        # centery = -data.info.origin.position.y*res*24

        # centerx = local_map.shape[0]/2 + 155
        # centery = local_map.shape[1]/2 - 90
        # local_map = local_map[
        #         centerx - home.shape[0]/2:centerx+home.shape[0]/2+1, 
        #         centery - home.shape[1]/2:centery+home.shape[1]/2+1]
    
        # test = home / 2 + local_map / 2;
        # now = rospy.Time.now()

    def CmdCallback(self, data):
        """ the dwa naviagtor only rotates and moves forward """
        self.travelled += math.fabs(data.linear.x)
        self.rotated += math.fabs(data.angular.z)

    def ClockCallback(self, data):
        secs = data.clock.secs
        if secs - self.last_logged  < 10:
            return
        self.last_logged = secs
        entry = {
                'time': secs,
                'cost': self.travelled,
                'quality': self.CalculateMapQuality(),
                'area': self.mapped_area,
                'completeness': self.CalculateMapCompleteness(),
                }
        self.data['data'].append(entry)
        with open(self.robotname + 'data.json', 'w') as outfile:
            json.dump(self.data, outfile)

    def CalculateMapQuality(self):
        return 0;

    def CalculateMapCompleteness(self):
        return 0;

    def OccupancyGridToMat(self, Occ):
        width = Occ.info.width
        height = Occ.info.height
        rospy.get_published_topics();
        local_map = np.zeros((width, height), dtype="uint8")
        for i in range(0, len(Occ.data)):
            if(Occ.data[i] == -1):
                value = 100
            else:
                value = Occ.data[i]
            local_map[i%width, int(i/width)] = 255 - value*255.0
        return local_map
    
    def runNode(self):
        rospy.init_node('metrics', anonymous=False)

        self.robotname = rospy.get_param("prefix")
        self.robotname = rospy.get_param("prefix")

        self.data = {
          'date': 2017,
          'experiment': 2017,
          'robot_name': 2017,
          'data': []
        }
    
        rospy.Subscriber("merged_map", OccupancyGrid, self.GridCallback)
        rospy.Subscriber("cmd_vel", Twist, self.CmdCallback)
        rospy.Subscriber("/clock", Clock, self.ClockCallback)
    
        rate = rospy.Rate(0.2) #hz
    
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        metrics = Metrics();
        metrics.runNode();
    except rospy.ROSInterruptException:
        pass

