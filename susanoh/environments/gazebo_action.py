#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import rospy
import rospy.exceptions
from geometry_msgs.msg import Twist


class Gazebo_Action:
    def __init__(self, robot_name="mobile_base"):
        # rostopic name for turtlebot
        topic_name = robot_name+'/commands/velocity'
        try:
            # they must be called
            rospy.init_node(robot_name+'_vel_publisher')
        except rospy.exceptions.ROSException as e:
            print(e)
        except rospy.exceptions.ROSInitException as e:
            print(e)

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)


    '''
    action(Int) define robot action
    0: robot don't move.
    1: robot move fowrard
    2:            backward
    3: robot rotate right
    4:              left
    '''
    def control_action(self,action):
    	self.move_to_neutral()
        if action == 0:
            self.move_to_neutral()
        elif action == 1:
            self.move_forward()
        elif action == 2:
            self.move_backword()
        elif action == 3:
            self.rotate_right()
        elif action == 4:
            self.rotate_left()
        else:
            rospy.loginfo("unexpected action=%d.",action)
            self.move_to_neutral()


    def move_to_neutral(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        self.pub.publish(vel)
        rospy.sleep(1.0)


    def move_forward(self):
        vel = Twist()
        vel.linear.x = 1.0
        self.pub.publish(vel)
        rospy.sleep(7.0)


    def move_backword(self):
        vel = Twist()
        vel.linear.x = -1.0
        self.pub.publish(vel)


    def rotate_right(self):
        vel = Twist()
        vel.angular.z = -2.0
        self.pub.publish(vel)


    def rotate_left(self):
        vel = Twist()
        vel.angular.z = 2.0
        self.pub.publish(vel)


def main():
    ga = Gazebo_Action()
    ga.control_action(4)
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
