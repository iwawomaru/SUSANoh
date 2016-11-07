#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import numpy as np

import rospy
import rospy.exceptions
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import time

import cv2
from cv_bridge import CvBridge, CvBridgeError



class GazeboAction:
    def __init__(self, target_name="player"):
        
        self.cv_image = None
        self.bridge = CvBridge()

        try:
            # they must be called
            rospy.init_node('vel_publisher_'+target_name)
        except rospy.exceptions.ROSException as e:
            print(e)
        except rospy.exceptions.ROSInitException as e:
            print(e)

        # rostopic name for turtlebot
        topic_name_vel = target_name+'/mobile_base/commands/velocity'
        self.pub = rospy.Publisher(topic_name_vel, Twist, queue_size=10)

        camera_name=target_name+"/camera"
        # self.topic_name_cam = camera_name+'/rgb/image_raw'
        self.topic_name_cam = camera_name+'/depth/image_raw'
        self.image_sub = rospy.Subscriber(self.topic_name_cam, Image, 
                                          self.image_callback)
        '''
        action(Int) define robot action
        0: robot don't move.
        1: robot move fowrard
        2:            backward
        3: robot rotate right
        4:              left
        '''
    def control_action(self,action):
        vel = Twist()
    	# self.move_to_neutral()
        if action == 0:
            # self.move_to_neutral()
            pass
        elif action == 1:
            # self.move_forward()
            vel.linear.x = 1.0
        elif action == 2:
            # self.move_backword()
            vel.linear.x = -1.0
        elif action == 3:
            # self.rotate_right()
            vel.angular.z = -2.0
        elif action == 4:
            # self.rotate_left()
            vel.angular.z = 2.0
        else:
            print("unexpected action=%d.",action)
            rospy.loginfo("unexpected action=%d.",action)
        
        self.pub.publish(vel)

    # return [[b, g, r],[b, g, r] ...]
    def get_image_array(self):
        return self.cv_image

    # subscriber callback function(this cause a bug)
    def image_callback(self, data):
        depth_array = None
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            depth_array = np.array(self.cv_image, dtype=np.float32)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        except CvBridgeError as e:
            print(e)

        ## if you need the image turtlebot sees, uncomment these
        # cv2.imshow("image window", self.cv_image)
        # print "cv_image", self.cv_image
        cv2.waitKey(3)
        if depth_array is not None:
            cv2.imwrite("pic/pic"+str(time.time())+".bmp", depth_array*255)

    def __del__(self):
        # Kill gzclient, gzserver and roscore                                   
        tmp = os.popen("ps -Af").read()
        gzclient_count = tmp.count('gzclient')
        gzserver_count = tmp.count('gzserver')
        roscore_count = tmp.count('roscore')
        rosmaster_count = tmp.count('rosmaster')
        if gzclient_count > 0:
            os.system("killall -9 gzclient")
        if gzserver_count > 0:
            os.system("killall -9 gzserver")
        if rosmaster_count > 0:
            os.system("killall -9 rosmaster")
        if roscore_count > 0:
            os.system("killall -9 roscore")

        if (gzclient_count or gzserver_count or 
            roscore_count or rosmaster_count >0):
            os.wait()

def test():
    ga = GazeboAction()
    ga2 = GazeboAction(target_name="keeper")
    for i in range(1000000):
        ga.control_action(4)
        ga2.control_action(3)
    # print(ga.get_image_array())
    rospy.spin()


if __name__ == "__main__":
    sys.exit(test())
