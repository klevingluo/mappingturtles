#!/usr/bin/env python

import rospy
import cv2
import tf
import json
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

class Metrics:
    """ a class for measuring the exploration progress """
    def __init__(self):
        self.data = []

    def GridCallback(self, data):
        local_map = self.OccupancyGridToMat(data)
        home = cv2.imread("../world/home.pgm", 0)
    
        listener = tf.TransformListener()
        rospy.sleep(0.5)
        position, quat = listener.lookupTransform("world", "robot_0/map", rospy.Time(0)) 
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = euler[2]
    
        local_map = cv2.copyMakeBorder(
                local_map, 
                0, 
                home.shape[1] - local_map.shape[1], 
                0, 
                home.shape[0] - local_map.shape[0], 
                cv2.BORDER_CONSTANT)
    
        rows,cols = local_map.shape
        # rotation is given by the rotation parameter in the world
        M = cv2.getRotationMatrix2D((cols/2,rows/2),-126.875,1)
        local_map = cv2.warpAffine(local_map,M,(cols,rows))
        # 20 is te resolution, 24 is the size of the map
        local_map = cv2.resize(
                local_map,
                (local_map.shape[0]/20*home.shape[0]/24, 
                    local_map.shape[1]/20*home.shape[1]/24))
    
    
        center = [[data.info.origin.position.x], [data.info.origin.position.y], [0]]
        center = np.dot(M,center)
        print center
    
        centerx = local_map.shape[0]/2  
        centery = local_map.shape[1]/2 
        local_map = local_map[
                centerx - home.shape[0]/2:centerx+home.shape[0]/2+1, 
                centery - home.shape[1]/2:centery+home.shape[1]/2+1]
    
        test = home / 2 + local_map / 2;
        now = rospy.Time.now()
        cv2.imshow('test', test)
        cv2.waitKey(0)

    """ calulates the area mapped """
    def CalculateMappedArea(self, Occ):
        mapped_area = 0;
        for i in range(0, len(Occ.data)):
            if(Occ.data[i] != -1 and Occ.data[i] < 50):
                mapped_area = mapped_area + 1
        return mapped_area

    def CalulateExplorationCost(self):
        return 0;

    def CalulateMapQuality(self):
        return 0;

    def CalulateExplorationTime(self):
        return 0;

    def CalulateEfficiency(self):
        return 0;

    def CalulateMapCompleteness(self):
        return 0;
    
    def WriteEntry(self):
        entry = {
                'timestamp': str(now),
                'quality': self.CalulateMapQuality(),
                'efficiency': self.CalulateEfficiency(),
                'completeness': self.CalulateMapCompleteness(),
                'area': self.CalculateMappedArea(),
                'cost': self.CalulateExplorationCost(),
        }
        self.data['data'].append(entry)
        with open('data.json', 'w') as outfile:
            json.dump(self.data, outfile)

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

        self.data = {
          'date': 2017,
          'experiment': 2017,
          'robot_name': 2017,
          'data': []
        }
    
        # prefix = rospy.get_param("prefix")
        rospy.Subscriber("robot_0/map", OccupancyGrid, self.GridCallback)
    
        rate = rospy.Rate(0.2) #hz
    
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        metrics = Metrics();
        metrics.runNode();
    except rospy.ROSInterruptException:
        pass

