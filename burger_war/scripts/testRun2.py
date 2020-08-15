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
import json

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

target_coordinate = np.array([
    [
	#[-1.0 ,-0.3 , 330],
	[-0.9 ,-0.4 , 340],
	[-0.6 , 0.0 ,   0],
	[-0.9 , 0.4 ,  20]
    ],[
	[ 0   , 0.5 , 180],
	[ 0   , 0.5 , 270],
	[ 0   , 0.5 ,   0]
    ],[
	[ 0.9 , 0.4 , 160],
	[ 0.6 , 0.0 , 180],
	[ 0.9 ,-0.4 , 200]
    ],[
	[ 0   ,-0.5 ,   0],
	[ 0   ,-0.5 ,  90],
	[ 0   ,-0.5 , 180]
    ]
])

target_idx = np.array([
    [ 13, 17, 11],
    [ 10, 16,  7],
    [  6, 14,  8],
    [  9, 15, 12]
])

# Priority of targets for each Zone
target_pri = np.array([
    [ 0, 2, 1],
    [ 2, 1, 0],
    [ 0, 2, 1],
    [ 2, 1, 0]
])

zone_border_coordinate = np.array([
    [
	#[-0.5 ,-0.1 , 315],  # Zone 0 -> Zone 3
	[-0.3 ,-0.3 , 315],  # Zone 0 -> Zone 3
	[-0.3 , 0.3 ,  45]   # Zone 0 -> Zone 1
    ],[
	[-0.3 , 0.3 , 225],  # Zone 1 -> Zone 0
	[ 0.3 , 0.3 , 315]   # Zone 1 -> Zone 2
    ],[
	[ 0.3 , 0.3 , 135],  # Zone 2 -> Zone 1
	[ 0.3 ,-0.3 , 225]   # Zone 2 -> Zone 3
    ],[
	[ 0.3 ,-0.3 ,  45],  # Zone 3 -> Zone 2
	[-0.3 ,-0.3 , 135]   # Zone 3 -> Zone 0
    ]
])

class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name
        self.name = bot_name
        self.my_side = rospy.get_param("side")
        print "SIDE:", self.my_side
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # navigation publisher
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # usb camera
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        topicname_image_raw = "image_raw"
        self.image_sub = rospy.Subscriber(topicname_image_raw, Image, self.imageCallback)

        self.basic_mode_process_step_idx = 0 # process step in basic MODE

        # war status
        topicname_war_state = "war_state"
        self.war_state = rospy.Subscriber(topicname_war_state, String, self.stateCallback)
        self.my_score = 0
        self.enemy_score = 0
        self.target_cnt = 0
        self.cur_zone = 0
        self.cur_target = target_pri[self.cur_zone][self.target_cnt]

	self.score = np.array([[1,1,1],[1,1,1],[1,1,1],[1,1,1]])

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

    def stateCallback(self, state):
        # print(state.data)
        dic = json.loads(state.data)

        if self.my_side == "r": # red_bot
            self.my_score = int(dic["scores"]["r"])
            self.enemy_score = int(dic["scores"]["b"])
        else: # blue_bot
            self.my_score = int(dic["scores"]["b"])
            self.enemy_score = int(dic["scores"]["r"])

	for zone in range(4):
	    for target in range(3):
		s = dic["targets"][target_idx[zone][target]]["player"]
		if s == "n":
		    self.score[zone][target] = 1
		elif s == self.my_side:
		    self.score[zone][target] = 0
		else:
		    self.score[zone][target] = 2
		#print dic["targets"][target_idx[zone][target]]["player"],
		print self.score[zone][target],
	    print

    def SetTwist(self,th):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def SearchMarker(self,t):
        r = rospy.Rate(10) # change speed 1fps
        #turn left
        th=0.25
        twist = self.SetTwist(th)
        for i in range(t):
            self.vel_pub.publish(twist)
            r.sleep()
        #stop
        th=0.0
        twist = self.SetTwist(th)
        self.vel_pub.publish(twist)
        time.sleep(1)
        #turn right
        th=-0.25
        twist = self.SetTwist(th)
        for i in range(t*2):
            self.vel_pub.publish(twist)
            r.sleep()
        #stop
        th=0.0
        twist = self.SetTwist(th)
        self.vel_pub.publish(twist)

        time.sleep(1)


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
	#################################
        #wait = self.client.wait_for_result()

        #if not wait:
        #    rospy.logerr("Action server not available!")
        #    rospy.signal_shutdown("Action server not available!")
        #    return -1

        ##print("SearchMarker")
        ##t=15
        ##self.SearchMarker(t)

        #get_state = self.client.get_state()
        #print("wait", wait, "get_state", get_state)
        #if get_state == 2:  # if send_goal is canceled
        #    return -1
	#################################

        return 0

    def cancelGoal(self):
        self.client.cancel_goal()

    def calcTwist(self):

        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(5) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        # ---> testrun
        while not rospy.is_shutdown():
            #NextGoal_coor = basic_coordinate[ self.basic_mode_process_step_idx ]
            NextGoal_coor = target_coordinate[self.cur_zone][self.cur_target]
            _x = NextGoal_coor[0]
            _y = NextGoal_coor[1]
            _th = NextGoal_coor[2] * DEGRAD
            ret = self.setGoal(_x, _y, _th)

	    while True:
		r.sleep()
		get_state = self.client.get_state()
		print get_state
		if self.score[self.cur_zone][self.cur_target] == 0:
		    self.client.cancel_goal()
		    while self.client.get_state() < 2:
			r.sleep()
		if get_state >= 2:
		    break

	    if self.score[self.cur_zone][self.cur_target] != 0:
		twist = Twist()
		twist.linear.x = 0.2; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		self.vel_pub.publish(twist)
		r.sleep()
		r.sleep()
		twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		self.vel_pub.publish(twist)
            #self.basic_mode_process_step_idx += 1
            #if self.basic_mode_process_step_idx >= len(basic_coordinate):
            #    self.basic_mode_process_step_idx = 0
            self.target_cnt += 1
            if self.target_cnt > 2:
		self.target_cnt = 0
		self.cur_zone += 1
		if self.cur_zone > 3:
		    self.cur_zone = 0
	    self.cur_target = target_pri[self.cur_zone][self.target_cnt]

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
