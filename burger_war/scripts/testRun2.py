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
import requests

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
DEGRAD = np.pi / 180

target_coordinate = np.array([
    [
	#[-1.0 ,-0.3 , 330],
	[-0.8 ,-0.4 , 340],
	[-0.5 , 0.0 ,   0],
	[-0.8 , 0.4 ,  20]
    ],[
	[ 0   , 0.5 , 180],
	[ 0   , 0.5 , 270],
	[ 0   , 0.5 ,   0]
    ],[
	[ 0.8 , 0.4 , 160],
	[ 0.5 , 0.0 , 180],
	[ 0.8 ,-0.4 , 200]
    ],[
	[ 0   ,-0.5 ,   0],
	[ 0   ,-0.5 ,  90],
	[ 0   ,-0.5 , 180]
    ]
])

target_idx_r = np.array([
    [ 13, 17, 11],
    [ 10, 16,  7],
    [  6, 14,  8],
    [  9, 15, 12]
])
target_idx_b = np.array([
    [  6, 14,  8],
    [  9, 15, 12],
    [ 13, 17, 11],
    [ 10, 16,  7]
])

# Priority of targets for each Zone
target_pri = np.array([
    [ 0, 2, 1],
    [ 0, 1, 2],
    [ 0, 2, 1],
    [ 0, 1, 2]
])

zone_border_coordinate = np.array([
    [
	#[-0.3 ,-0.3 , 315],  # Zone 0 -> Zone 3
	#[-0.3 , 0.3 ,  45]   # Zone 0 -> Zone 1
	[-0.4 ,-0.2 , 315],  # Zone 0 -> Zone 3
	[-0.4 , 0.2 ,  45]   # Zone 0 -> Zone 1
    ],[
	#[-0.3 , 0.3 , 225],  # Zone 1 -> Zone 0
	#[ 0.3 , 0.3 , 315]   # Zone 1 -> Zone 2
	[-0.2 , 0.4 , 225],  # Zone 1 -> Zone 0
	[ 0.2 , 0.4 , 315]   # Zone 1 -> Zone 2
    ],[
	#[ 0.3 , 0.3 , 135],  # Zone 2 -> Zone 1
	#[ 0.3 ,-0.3 , 225]   # Zone 2 -> Zone 3
	[ 0.4 , 0.2 , 135],  # Zone 2 -> Zone 1
	[ 0.4 ,-0.2 , 225]   # Zone 2 -> Zone 3
    ],[
	#[ 0.3 ,-0.3 ,  45],  # Zone 3 -> Zone 2
	#[-0.3 ,-0.3 , 135]   # Zone 3 -> Zone 0
	[ 0.2 ,-0.4 ,  45],  # Zone 3 -> Zone 2
	[-0.2 ,-0.4 , 135]   # Zone 3 -> Zone 0
    ]
])

class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name
        self.name = bot_name
        self.my_side = rospy.get_param("side")
        print "SIDE:", self.my_side
        print "JUDGE_URL:", JUDGE_URL
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # navigation publisher
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # Lidar
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # odom
        topicname_odom = "odom"
        self.odom = rospy.Subscriber(topicname_odom, Odometry, self.odomCallback)

        # amcl pose
        topicname_amcl_pose = "amcl_pose"
        self.amcl_pose = rospy.Subscriber(topicname_amcl_pose, PoseWithCovarianceStamped, self.AmclPoseCallback)

        # usb camera
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        topicname_image_raw = "image_raw"
        self.image_sub = rospy.Subscriber(topicname_image_raw, Image, self.imageCallback)

        self.basic_mode_process_step_idx = 0 # process step in basic MODE

        self.scan_ave = np.zeros((2,12))        # [0]:latest, [1]:prev
        self.myPosX = 0
        #self.myPosY = -150
        self.myPosY = -130
        #self.myDirect = np.pi / 2
        self.myYawA = 0
        self.myYawB = 0
	self.myPos = np.zeros((4,10))

        # war status
        self.my_score = 0
        self.enemy_score = 0
        self.target_cnt = 0
	self.cur_zone = 0
	self.enemy_zone = 2
        self.nxt_zone = self.cur_zone
        self.nxt_idx = 0
        self.cur_target = target_pri[self.cur_zone][self.target_cnt]

	self.score = np.array([[1,1,1],[1,1,1],[1,1,1],[1,1,1]])
	self.score_prev = np.array([[1,1,1],[1,1,1],[1,1,1],[1,1,1]])
	self.enemy_stamp = rospy.Time.now()

    def odomCallback(self, data):
        # print(data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        e = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
        # print(e[2] / (2 * np.pi) * 360)
        self.myYawA = e[2] / DEGRAD - 90
	if self.myYawA < 0:
	    self.myYawA += 360
        self.myYawB = e[2] / DEGRAD + 90
	if self.myYawB < 0:
	    self.myYawB += 360

    def AmclPoseCallback(self, data):
        self.myPosX = data.pose.pose.position.x * 100
        self.myPosY = data.pose.pose.position.y * 100
        # print(self.myPosX, self.myPosY)

    # Lidar
    def lidarCallback(self, data):
        self.scan = data

        self.scan_ave[1] = self.scan_ave[0]        # prev <= latest
        self.scan_ave[0,0] = (sum(self.scan.ranges[0:2])+sum(self.scan.ranges[358:359])) * 200 # /5 * 1000
        if self.scan_ave[0,0] == float('inf'):
            self.scan_ave[0,0] = 100
        i = 1
        while i < 12:
            self.scan_ave[0,i] = sum(self.scan.ranges[i*30-2:i*30+2]) * 200 # /5 * 1000
            if self.scan_ave[0,i] == float('inf'):
                self.scan_ave[0,i] = 100
            i += 1
        #self.scan_diff = self.scan_ave[0] - self.scan_ave[1]

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
            #print("image show")
            cv2.imshow("Image window", frame)
            cv2.waitKey(1)

    def getWarState(self):
	resp = requests.get(JUDGE_URL + "/warState")
	dic = resp.json()
        if self.my_side == "r": # red_bot
            self.my_score = int(dic["scores"]["r"])
            self.enemy_score = int(dic["scores"]["b"])
        else: # blue_bot
            self.my_score = int(dic["scores"]["b"])
            self.enemy_score = int(dic["scores"]["r"])

	for zone in range(4):
	    for target in range(3):
		if self.my_side == "r":
		    target_idx = target_idx_r[zone][target]
		else:
		    target_idx = target_idx_b[zone][target]

		s = dic["targets"][target_idx]["player"]
		self.score_prev[zone][target] = self.score[zone][target]
		if s == "n":
		    self.score[zone][target] = 1
		elif s == self.my_side:
		    self.score[zone][target] = 0
		else:
		    self.score[zone][target] = 2
		    if self.score_prev[zone][target] < 2:	# When the enemy get a marker,
			self.enemy_zone = zone
			self.enemy_stamp = rospy.Time.now()
			print "#",
		print self.score[zone][target],
	    print "/",

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

    def nextZone(self, cur_zone):
	for zone in range(4):
	    #s[zone] = np.sum(self.score[zone])
	    s = np.sum(self.score, axis=1)
	left = cur_zone + 1
	if left > 3:
	    left = 0
	right = cur_zone - 1
	if right < 0:
	    right = 3
	#s[left] = s[left]+1

	tm = rospy.Time.now()
	print "Zone", self.enemy_zone, "Stamp", self.enemy_stamp.secs, "Now", tm.secs
	ofs = 6 - (tm.secs - self.enemy_stamp.secs) * 0.2
	if ofs >= 0:
	    s[self.enemy_zone] -= ofs

	print "nextZone", s
	if s[left] >= s[right]:
	    return left,1
	else:
	    return right,0

    def strategy(self):
        r = rospy.Rate(5) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
	mode = 0
	stack_count = 0
	time_setGoal = rospy.Time.now()
	f_skip = False

        # ---> testrun
        while not rospy.is_shutdown():
	    if self.target_cnt < 3:
		NextGoal_coor = target_coordinate[self.cur_zone][self.cur_target]
	    else:
		NextGoal_coor = zone_border_coordinate[self.cur_zone][self.nxt_idx]
		self.cur_zone = self.nxt_zone
            _x = NextGoal_coor[0]
            _y = NextGoal_coor[1]
            _th = NextGoal_coor[2] * DEGRAD
            ret = self.setGoal(_x, _y, _th)
	    time_setGoal = rospy.Time.now()
	    f_skip = False

	    while True:
		r.sleep()
		self.getWarState()
		get_state = self.client.get_state()
		print get_state, self.target_cnt, self.cur_zone,
		cur_time = rospy.Time.now()
		f_skip = (cur_time.secs - time_setGoal.secs >= 15)
		if get_state >= 2:
		    break
		if self.score[self.cur_zone][self.cur_target] == 0 or f_skip:
		    self.client.cancel_goal()
		    while self.client.get_state() < 2:
			r.sleep()

		# Calc current position
		self.myPos = np.roll(self.myPos,1)
		self.myPos[0][0] = self.myPosX
		self.myPos[1][0] = self.myPosY
		self.myPos[2][0] = self.myYawA
		self.myPos[3][0] = self.myYawB

		print " myPos", '{:.1f}'.format(self.myPosX), '{:.1f}'.format(self.myPosY), '{:.1f}'.format(self.myYawA), '{:.1f}'.format(self.myYawB),
		myPos_std = np.std(self.myPos, axis=1)
		print "stdout", '{:.1f}'.format(myPos_std[0]), '{:.1f}'.format(myPos_std[1]), '{:.1f}'.format(myPos_std[2]), '{:.1f}'.format(myPos_std[3]), stack_count,(cur_time.secs - time_setGoal.secs)
		if myPos_std[2] < myPos_std[3]:
			myPos_std23 = myPos_std[2]
		else:
			myPos_std23 = myPos_std[3]
		if (myPos_std[0] < 1.0) and (myPos_std[1] < 1.0) and (myPos_std23 < 5.0):
		    stack_count += 1
		elif stack_count > 0:
		    stack_count -= 1
		else:
		    stack_count = 0
		# Recovery from stacking
		if stack_count > 5:
		    stack_count = 0
		    twist = Twist()
		    turn_left = False
		    turn_right = False
		    if self.scan_ave[0,9] < 250:
			if self.scan_ave[0,3] > 300:
			    turn_left = True
		    if self.scan_ave[0,3] < 250:
			if self.scan_ave[0,9] > 300:
			    turn_right = True

		    if self.scan_ave[0,0] > 300 or self.scan_ave[0,0] > self.scan_ave[0,6]:
			twist.linear.x = 0.2
			if turn_left:
			    twist.angular.z = 0.5
			if turn_right:
			    twist.angular.z = -0.5
		    else:
			twist.linear.x = -0.2
			if turn_left:
			    twist.angular.z = -0.5
			if turn_right:
			    twist.angular.z = 0.5
		    twist.linear.y = 0; twist.linear.z = 0
		    twist.angular.x = 0; twist.angular.y = 0;
		    self.vel_pub.publish(twist)
		    print "Stack Recovery", "F",self.scan_ave[0,0], "B",self.scan_ave[0,6], "L",self.scan_ave[0,3], "R",self.scan_ave[0,9], twist.linear.x, twist.angular.z
		    r.sleep()
		    r.sleep()
		    r.sleep()
		    r.sleep()
		    twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
		    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		    self.vel_pub.publish(twist)
		    print "End Recovery"
	    #end while

	    if f_skip:
		    print "SKIPPED."
	    if self.target_cnt < 3 and (not f_skip):
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

	    if self.target_cnt < 3:
		self.target_cnt += 1
	    else:
		self.target_cnt = 0
	    while self.target_cnt < 3:
		self.cur_target = target_pri[self.cur_zone][self.target_cnt]
		if self.score[self.cur_zone][self.cur_target] != 0:
		    break
		self.target_cnt += 1

	    if self.target_cnt == 3:
		# Next Zone
		self.nxt_zone, self.nxt_idx = self.nextZone(self.cur_zone)
	

        # ---< testrun

        #while not rospy.is_shutdown():
        #    twist = self.calcTwist()
        #    print(twist)
        #    self.vel_pub.publish(twist)

        #    r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_run')
    JUDGE_URL = rospy.get_param('/send_id_to_judge/judge_url', 'http://127.0.0.1:5000')

    bot = RandomBot('Random')
    bot.strategy()
