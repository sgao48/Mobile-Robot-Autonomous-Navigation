#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

import math

class GazeboLinkPose:
    link_name = ''
    link_pose = Pose()
    real_pose = Pose()

    def __init__(self, robot_name, link_name):
        self.robot_name = robot_name
        self.link_name = link_name
        # self.link_name_rectified = link_name.replace("::", "_")

        if not self.link_name:
            raise ValueError("'link_name' is an empty string")

        self.states_sub = rospy.Subscriber(
           "/gazebo/link_states", LinkStates, self.callback)
        self.pose_pub = rospy.Publisher(
            "/gazebo/" + (self.robot_name + '__' + self.link_name), Pose, queue_size=10)
        self.tf_pub = tf.TransformBroadcaster()
        
        # subsrcibe the ekf info
        self.ekf_sub = rospy.Subscriber("/ekf_slam", Pose, self.ekf_callback)
        self.isFirst = 1
        # error related
        self.goal_sub = rospy.Subscriber("/course_agv/goal", PoseStamped, self.goal_callback)
        self.zero, self.now, self.over = None, 0.0, None
        self.time_N = -1
        self.goal_x, self.goal_y = 999999, 999999

        self.error_pub = rospy.Publisher("close_error", Float64, queue_size=10)

    # goal callback
    def goal_callback(self, data):
        if self.zero is None:
            self.zero = rospy.Time.now().to_sec()
            print("----------ZERO SET----------")

            self.goal_x = data.pose.position.x
            self.goal_y = data.pose.position.y

    # ekf callback
    def ekf_callback(self, data):
        self.link_pose = data
        if self.isFirst:
           print("----------EKF START----------")
           self.isFirst = 0
    
    def callback(self, data):
        try:
            ind = data.name.index(self.robot_name + "::" + self.link_name)
            self.real_pose = data.pose[ind]
        except ValueError:
            pass
    
    def publish_tf(self):
        p = self.link_pose.position
        o = self.link_pose.orientation
        rp = self.real_pose.position
        ro = self.real_pose.orientation
        
        if self.isFirst:    # no location message
            self.tf_pub.sendTransform((rp.x, rp.y, rp.z),(ro.x, ro.y, ro.z, ro.w),
                                        rospy.Time.now(),self.link_name,"map")
        else:
            self.tf_pub.sendTransform((p.x, p.y, p.z),(o.x, o.y, o.z, o.w),
                                        rospy.Time.now(),self.link_name,"map")
            # output error
            self.now = rospy.Time.now().to_sec()
            if self.zero is not None and self.over is None:
                dis = math.hypot(rp.x-p.x, rp.y-p.y)
                self.error_pub.publish(dis)

                real_time = self.now - self.zero
                if int(real_time/0.1) != self.time_N:
                    print("Time: %.1f distance: %.3f" % (real_time, dis))
                    self.time_N += 1

                # reach goal
                goal_dis = math.hypot(self.goal_x-p.x, self.goal_y-p.y)
                if goal_dis <= 0.2:
                    self.over = rospy.Time.now().to_sec()
                    print("----------OVER SET----------")

if __name__ == '__main__':
    try:
        rospy.init_node('robot_pose_tf_publisher')
        gp = GazeboLinkPose(rospy.get_param('~robot_name', 'course_agv'),
                            rospy.get_param('~link_name', 'robot_base'))
        publish_rate = rospy.get_param('~publish_rate', 10)

        rate = rospy.Rate(publish_rate)
        while not rospy.is_shutdown():
            gp.pose_pub.publish(gp.link_pose)
            gp.publish_tf()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
