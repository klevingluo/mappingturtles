#!/usr/bin/env python

import tf
import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler 
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from geometry_msgs.msg import Quaternion, PoseStamped

class Explorer:

    pub = 2;
    frontier_publisher = 2;
    frontiers = [];
    pose = 3
    nav_goal = 0

    def __init__(self):
        self.frontiers = []
        self.frontierFrames = 0

    def OdomCallback(self, data):
        self.pose = data.pose.pose

    def GridCallback(self, data):
        rospy.logwarn("got a new map")
        local_map = self.OccupancyGridToMat(data)

        newFrontiers = []
        for x in range(0, np.size(local_map,0),5):
            for y in range(0, np.size(local_map,1),5):
                if (local_map[x,y] != 255):
                    continue;
                count = 0
                rays = [
                        lambda i: local_map[x, y + i],
                        lambda i: local_map[x + i, y + i],
                        lambda i: local_map[x + i, y],
                        lambda i: local_map[x + i, y - i],
                        lambda i: local_map[x, y - i],
                        lambda i: local_map[x - i, y - i],
                        lambda i: local_map[x - i, y],
                        lambda i: local_map[x - i, y + i],
                ]
                raylengths = [];
                for i in rays:
                    raycount = 0
                    for j in range(0,16):
                        if i(j) == 0 and j < 8:
                            raycount = -1
                            break
                        elif i(j) == 100 and j < 8:
                            raycount = -1
                            break
                        elif i(j) == 100:
                            raycount = raycount + 1
                        elif i(j) == 0:
                            break
                    if raycount == -1:
                        count = 0
                        raylengths.append(0)
                        break
                    else:
                        count += raycount
                        raylengths.append(raycount)

                if count > 0:
                    (x,y) = self.convertToWorld(x,y,data)
                    angle = 3.14 / 4 * np.argmax(raylengths) + 3.14
                    newFrontiers.append((x,y,angle))

        def distanceToFrontier(frontier):
            point = np.array([ frontier[0], frontier[1]])
            point2 = np.array([self.pose.position.x, self.pose.position.y])
            return np.linalg.norm(point - point2)

        newFrontiers = sorted(newFrontiers, key=distanceToFrontier)
        newFrontiers = filter(lambda x: distanceToFrontier(x) > 1, newFrontiers)
        self.frontiers = newFrontiers
        self.publish_frontiers()
        return

    def navigateToGoal(self, clock):
        if len(self.frontiers) < 1:
            return
        self.nav_goal = self.create_nav_goal(
                self.frontiers[0][0], 
                self.frontiers[0][1], 
                self.frontiers[0][2])
        self.frontiers.pop()
        self.publish_nav_goal(self.nav_goal)
        if (self.nav_goal):
            nav_as = actionlib.SimpleActionClient(
                    'move_base', 
                    MoveBaseAction)
            rospy.loginfo("Connecting to /move_base AS...")
            nav_as.wait_for_server()
            rospy.loginfo("Connected.")
            nav_as.send_goal(self.nav_goal)
            rospy.loginfo("Waiting for result...")
            # waits for 10 seconds
            nav_as.wait_for_result(rospy.Duration(10))
            nav_res = nav_as.get_result()
            nav_state = nav_as.get_state()
            rospy.loginfo("Done!")
            print "Result: ", str(nav_res) # always empty, be careful
            print "Nav state: ", str(nav_state) 
            # use this, 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)

    """ converts coordinates from map coordinates to the world frame """
    def convertToWorld(self, x, y, grid):
        newx = x * grid.info.resolution + grid.info.origin.position.x + grid.info.resolution / 2.0;
        newy = y * grid.info.resolution + grid.info.origin.position.y + grid.info.resolution / 2.0;
        return (newx, newy)

    def publish_nav_goal(self, nav_goal):
        self.pub.publish(nav_goal.target_pose);

    def publish_frontiers(self): 

        def make_pose(frontier):
            pose = Pose()
            quat = quaternion_from_euler(0.0, 0.0, frontier[2]) # roll, pitch, yaw
            pose.orientation = Quaternion(*quat.tolist())
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            pose.position.x = frontier[0]
            pose.position.y = frontier[1]
            pose.position.z = 0
            return pose

        poseArray = PoseArray()
        poseArray.header.frame_id = self.robotname + "/map"
        poseArray.header.stamp = rospy.Time()
        poseArray.header.seq = self.frontierFrames
        self.frontierFrames += 1 
        poseArray.poses = map(make_pose, self.frontiers)
        self.frontier_publisher.publish(poseArray)

    def create_nav_goal(self, x, y, yaw):
        """Create a MoveBaseGoal with x, y position and yaw rotation (in degrees).
        Returns a MoveBaseGoal"""
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = self.robotname + "/map" # Note: the frame_id must be map
        mb_goal.target_pose.pose.position.x = x
        mb_goal.target_pose.pose.position.y = y
        mb_goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)

        # Orientation of the robot is expressed in the yaw value of euler angles
        angle = yaw # angles are expressed in radians, the conversion is done elsewhere
        quat = quaternion_from_euler(0.0, 0.0, yaw) # roll, pitch, yaw
        mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

        return mb_goal
        
    def OccupancyGridToMat(self, Occ):
        width = Occ.info.width
        height = Occ.info.height
        rospy.get_published_topics();
        local_map = np.zeros((width, height), dtype="uint8")
        for i in range(0, len(Occ.data)):
            value = 255
            if(Occ.data[i] == -1):
                value = 100
            elif(Occ.data[i] > 0.5):
                value = 0
            local_map[i%width, int(i/width)] = value
        return local_map
    
    def runNode(self):
        rospy.init_node(
                "explorer", 
                anonymous=False)

        self.robotname = rospy.get_param("prefix")

        self.pub = rospy.Publisher(
                "goal", 
                PoseStamped, 
                queue_size=10)

        self.frontier_publisher = rospy.Publisher(
                "frontiers", 
                PoseArray, 
                queue_size=10)

        # prefix = rospy.get_param("prefix")
        rospy.Subscriber(
                "map", 
                OccupancyGrid, 
                self.GridCallback)

        rospy.Subscriber(
                "odom", 
                Odometry, 
                self.OdomCallback)

        rospy.Timer(rospy.Duration(10), self.navigateToGoal)
    
        rate = rospy.Rate(0.2) #hz
    
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        explorer = Explorer();
        explorer.runNode();
    except rospy.ROSInterruptException:
        pass
