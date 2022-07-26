#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import numpy
import math

from nav_msgs.msg import OccupancyGrid
from course_agv_nav.srv import Plan, PlanResponse
from threading import Lock,Thread

MAX_LASER_RANGE = 30

class Checker():
    def __init__(self):
        self.threshold = 0.1 # use to check collision
        self.path = numpy.zeros([3,0])
        # x y yaw
        self.robot_x = numpy.zeros([3,1])
        # ros topic
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
	    # global real pose TODO replace with slam pose
        self.pose_sub = rospy.Subscriber('/gazebo/course_agv__robot_base',Pose,self.poseCallback)

        self.map_sub = rospy.Subscriber('/grid_map_mine', OccupancyGrid, self.mapCallback)
        self.replan_client = rospy.ServiceProxy('/course_agv/global_plan', Plan)
        self.flag = True
        self.delta = 2
        self.replanTimes = 0

        self.lock = Lock()

    def poseCallback(self,msg):
        p = msg.position
        o = msg.orientation
        e = tf.transformations.euler_from_quaternion([o.x,o.y,o.z,o.w])
        self.robot_x = numpy.array([[p.x,p.y,e[2]]]).T

    def pathCallback(self,msg):
        self.path = self.pathToNumpy(msg) # shape (3,pose_num)

    def mapCallback(self, msg):
        if(self.flag):
            self.height = msg.info.height
            self.width = msg.info.width
            self.reso = msg.info.resolution
            self.flag = False
        self.map = numpy.array(msg.data).reshape((-1,msg.info.height)).transpose()
        if self.collision_check():
            try:
                res = self.replan_client.call()
                self.replanTimes += 1
                print(self.replanTimes)
            except rospy.ServiceException, e:
                pass

    def collision_check(self):
        if self.path.shape[1] == 0:
            return False

	    ## TODO
        for i in range(self.path.shape[1]):
            x = self.poseToInd(self.path[0, i])
            y = self.poseToInd(self.path[1, i])
            xl, xr = max(x-self.delta, 0), min(x+self.delta, self.height)
            yl, yr = max(y-self.delta, 0), min(y+self.delta, self.width)

            if numpy.array(self.map[xl:xr+1,yl:yr+1]>50).max():
                return True

        return False

    def poseToInd(self, x):
        return int((x+10.0) / self.reso)

    def pathToNumpy(self,msg):
        pos = numpy.ones([0,3])
        for i in range(len(msg.poses)):
            p = msg.poses[i].pose.position
            pos = numpy.append(pos,[[p.x,p.y,1]],axis=0)
        return pos.T

    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = numpy.array([[t[0,2],t[1,2],dw]])
        return u.T
    
    def u2T(self,u):
        w = u[2]
        dx = u[0]
        dy = u[1]

        return numpy.array([
            [ math.cos(w),-math.sin(w), dx],
            [ math.sin(w), math.cos(w), dy],
            [0,0,1]
        ])

def main():
    rospy.init_node('collision_checker_node')
    n = Checker()
    rospy.spin()

if __name__ == '__main__':
    main()
