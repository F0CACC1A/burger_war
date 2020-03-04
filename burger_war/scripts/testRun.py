#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf

import numpy as np
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import actionlib_msgs

# camera image 640*480
img_w = 640
img_h = 480
image_resize_scale = 1 # 8

# PI
PI = 3.1415
DEGRAD = 3.141592/180

# robot running coordinate in BASIC MODE
basic_coordinate = np.array([
    # x,    y,    th(deg)
    [-1.0 , 0.3 ,  30],  # 1
    [-1.0 ,-0.3 , 330],  # 2
    [-0.6 , 0.0 ,   0],  # 3
    [-0.5 ,-0.1 , 315],  # 4
    [ 0   ,-0.6 , 180],  # 5
    [ 0   ,-0.6 ,  90],  # 6
    [ 0   ,-0.5 ,   0],  # 7
    [ 0.5 ,-0.1 ,  45],  # 10

    [ 1.0 ,-0.3 , 210],  # 1
    [ 1.0 , 0.3 , 150],  # 2
    [ 0.6 , 0.0 , 180],  # 3
    [ 0.5 , 0.1 , 135],  # 4
    [ 0   , 0.6 ,   0],  # 5
    [ 0   , 0.6 , 270],  # 6
    [ 0   , 0.5 , 180],  # 7
    [-0.5 , 0.1 , 225]]  # 10
)
#    [-0.4, 0.0, 0],  # 1
#    [-0.9, 0.0, 0],  # 2
#    [-0.9, 0.4, 0],  # 3
#    [-0.9, -0.4, 0], # 4
#    [-0.9, 0.0, 0],  # 5
#    [0, -0.5, 0],    # 6
#    [0, -0.5, PI],   # 7
#    [0, -0.5, PI/2], # 8
#    [0, -1.2, PI/2]] # 17

class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # navigation publisher
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # usb camera
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        topicname_image_raw = "image_raw"
        self.image_sub = rospy.Subscriber(topicname_image_raw, Image, self.imageCallback)

        self.basic_mode_process_step_idx = 0 # process step in basic MODE

	self.scan_r = 0
	self.scan_rf = 0
	self.scan_rb = 0
	self.scan_f = 0
	self.prev_r = 0
	self.prev_rf = 0
	self.prev_rb = 0
	self.prev_f = 0

    # camera image call back sample
    # convert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        size = (img_w/image_resize_scale, img_h/image_resize_scale)
        frame = cv2.resize(self.img, size)

        if self.camera_preview:
            print("image show")
            cv2.imshow("Image window", frame)
            cv2.waitKey(1)

    # Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
    # Ref: https://github.com/hotic06/burger_war/blob/master/burger_war/scripts/navirun.py
    # RESPECT @hotic06
    # do following command first.
    #   $ roslaunch burger_navigation multi_robot_navigation_run.launch
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return -1

        get_state = self.client.get_state()
        print("wait", wait, "get_state", get_state)
        if get_state == 2:  # if send_goal is canceled
            return -1

        return 0

    def cancelGoal(self):
        self.client.cancel_goal()

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data

        # visualize scan data with radar chart
#        angles = np.linspace(0, 2 * np.pi, len(self.scan.ranges) + 1, endpoint=True)
#        values = np.concatenate((self.scan.ranges, [self.scan.ranges[0]]))
#        ax = self.lidarFig.add_subplot(111, polar=True)
#        ax.cla()
#        ax.plot(angles, values, 'o-')
#        ax.fill(angles, values, alpha=0.25)
#        ax.set_rlim(0, 3.5)
        # self.front_distance = self.scan.ranges[0]
#        self.front_distance = min(min(self.scan.ranges[0:10]),min(self.scan.ranges[350:359]))
#        self.front_scan = (sum(self.scan.ranges[0:4])+sum(self.scan.ranges[355:359])) / 10
#        self.back_distance = (min(self.scan.ranges[170:190]))
#        self.back_scan = (sum(self.scan.ranges[176:185])) / 10
	self.prev_f  = self.scan_f
	self.prev_rf = self.scan_rf
	self.prev_r  = self.scan_r
	self.prev_rb = self.scan_rb
        self.scan_f  = (sum(self.scan.ranges[0:2])+sum(self.scan.ranges[358:359])) / 5
        self.scan_rf = (sum(self.scan.ranges[298:302])) / 5
        self.scan_r  = (sum(self.scan.ranges[268:272])) / 5
        self.scan_rb = (sum(self.scan.ranges[238:242])) / 5
        if self.scan_f  == float('inf'):
	    self.scan_f  = 0.1
        if self.scan_rf == float('inf'):
	    self.scan_rf = 0.1
        if self.scan_r  == float('inf'):
	    self.scan_r  = 0.1
        if self.scan_rb == float('inf'):
	    self.scan_rb = 0.1
        #print("Lider", self.scan_rf, self.scan_r, self.scan_rb)

        # RESPECT @koy_tak        
#	if (self.scan.ranges[0] != 0 and self.scan.ranges[0] < DISTANCE_TO_WALL_THRESHOLD) or (self.scan.ranges[10] != 0 and self.scan.ranges[10] < DISTANCE_TO_WALL_THRESHOLD) or (self.scan.ranges[350] != 0 and self.scan.ranges[350] < DISTANCE_TO_WALL_THRESHOLD):
#            self.f_isFrontBumperHit = True
#            print("self.f_isFrontBumperHit = True")
#            self.cancelGoal()
#        else:
#            self.f_isFrontBumperHit = False

    def calcTwist(self):
	df = (self.scan_r + self.scan_rf + self.scan_rb) - (self.prev_r + self.prev_rf + self.prev_rb)
        print("Lider", self.scan_rf, self.scan_r, self.scan_rb, df)
	if self.scan_f < 0.1:
	    x = -0.1
	    th = 0
	#elif self.scan_rf > self.scan_r * 1.2:
	elif self.scan_rf - self.scan_rb > 0.1:
	    x = 0
	    th = -0.5
	#elif self.scan_rb > self.scan_r * 1.2:
	elif self.scan_rf - self.scan_rb < -0.1:
	    x = 0
	    th = 0.5
	elif self.scan_f < 0.15 or self.scan_rf < self.scan_r:
	    x = 0
	    th = 1.5
	else:
	    x = 0.22
	    # feedback control
	    K1 = -3.0
	    K2 = -1.0
	    th = (self.scan_r - 0.15) * K1 + df * K2

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        # ---> testrun
        #while not rospy.is_shutdown():
        #    NextGoal_coor = basic_coordinate[ self.basic_mode_process_step_idx ]
        #    _x = NextGoal_coor[0]
        #    _y = NextGoal_coor[1]
        #    _th = NextGoal_coor[2] * DEGRAD
        #    ret = self.setGoal(_x, _y, _th)
        #    self.basic_mode_process_step_idx += 1
        #    if self.basic_mode_process_step_idx >= len(basic_coordinate):
        #        self.basic_mode_process_step_idx = 0
        # ---< testrun
            
        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()

