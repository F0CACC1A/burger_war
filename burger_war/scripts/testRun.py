#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random

from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import actionlib_msgs

from geometry_msgs.msg import Twist

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

        # usb camera
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        topicname_image_raw = "image_raw"
        self.image_sub = rospy.Subscriber(topicname_image_raw, Image, self.imageCallback)

        self.basic_mode_process_step_idx = 0 # process step in basic MODE

    # camera image call back sample
    # convert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        size = (img_w/image_resize_scale, img_h/image_resize_scale)
        frame = cv2.resize(self.img, size)

        ###
        #
        # ここに観音さんスペシャル加える
        #
        ###

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
            NextGoal_coor = basic_coordinate[ self.basic_mode_process_step_idx ]
            _x = NextGoal_coor[0]
            _y = NextGoal_coor[1]
            _th = NextGoal_coor[2] * DEGRAD
            ret = self.setGoal(_x, _y, _th)
            self.basic_mode_process_step_idx += 1
            if self.basic_mode_process_step_idx >= len(basic_coordinate):
                self.basic_mode_process_step_idx = 0
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

