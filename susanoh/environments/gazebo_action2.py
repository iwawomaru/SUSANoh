#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import numpy as np

import rospy
import rospy.exceptions
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError



class GazeboAction:
    def __init__(self, target_name=""):

        self.cv_image = None
        self.bridge = CvBridge()

        try:
            rospy.init_node(target_name+'vel_publisher')
        except rospy.exceptions.ROSException as e:
            print(e)
        except rospy.exceptions.ROSInitException as e:
            print(e)

        topic_name_vel = target_name+'/mobile_base/commands/velocity'
        self.pub = rospy.Publisher(topic_name_vel, Twist, queue_size=10)

        camera_name=target_name+"camera"
        self.topic_name_cam = camera_name+'/rgb/image_raw'
        self.image_sub = rospy.Subscriber(self.topic_name_cam, Image, 
                                          self.image_callback)

    def control_action(self,action):
        '''
        action(Int) define robot action
        0: robot don't move.
        1: robot move fowrard
        2:            backward
        3: robot rotate right
        4:              left
        '''

        vel = Twist()
        if action == 0:
            pass
        elif action == 1:
            vel.linear.x = 1.0
        elif action == 2:
            vel.linear.x = -1.0
        elif action == 3:
            vel.angular.z = -2.0
        elif action == 4:
            vel.angular.z = 2.0
        else:
            rospy.loginfo("unexpected action=%d.",action)
        self.pub.publish(vel)

    # return [[b, g, r],[b, g, r] ...]
    def get_image_array(self):
        return self.cv_image

    # subscriber callback function(this cause a bug)
    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        ## if you need the image turtlebot sees, uncomment these

        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2GRAY)
        self.cv_image[:,:,0] = gray
        self.cv_image[:,:,1] = gray
        self.cv_image[:,:,2] = gray
        cv2.imshow("image window", self.cv_image)
        # print "cv_image", self.cv_image
        cv2.waitKey(3)
        # cv2.imwrite("pic.bmp", self.cv_image)

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

def main():
    ga = GazeboAction()
    ga.control_action(1)
    print(ga.get_image_array())
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
