#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import std_srvs.srv
from gazebo_ros import gazebo_interface
from gazebo_msgs import SpawnModel

# add soccer ball
def add_ball():
	'''
	TODO: to add new ball in gazebo field

	REFERENCE: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/indigo-devel/gazebo_ros/scripts/spawn_model
	'''
    rospy.init_node('spawn_ball')


# get ball location(x,y)
def get_ball_location(number=0):
	'''
	TODO: to get the parameter function assigns(number) of some balls 
	'''
	pass


# reset gazebo world(it resets models)
def reset_world():
    rospy.wait_for_service('gazebo/reset_world')
    try:
        # ServiceProxy and call means `rosservice call /gazebo/reset_world`
        srv = rospy.ServiceProxy('gazebo/reset_world', std_srvs.srv.Empty)
        srv.call()
        rospy.loginfo("reset world")
    except rospy.ServiceExceptions as e:
        print("Service call failed %s"%e)


# reset gazebo simulation(it resets1 models and **time**)
def reset_simulation():
    rospy.wait_for_service('gazebo/reset_simulation')
    try:
        # ServiceProxy and call means `rosservice call /gazebo/simulation_world`
        srv = rospy.ServiceProxy('gazebo/reset_simulation', std_srvs.srv.Empty)
        srv.call()
        rospy.loginfo("reset simulation")
    except rospy.ServiceExceptions as e:
        print("Service call failed %s"%e)


def spawn_turtlebot():
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    sp = SpawnModel
    sp.model_name = 'turtlebot'
    try:
        srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', std_srvs.srv.Empty)
        srv.call()