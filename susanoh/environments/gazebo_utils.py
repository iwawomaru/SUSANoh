#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import std_srvs.srv
from geometry_msgs.msg import Pose
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import SpawnModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

# add soccer ball
def add_ball():
    '''
    TODO: to add new ball in gazebo field
    
    REFERENCE: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/indigo-devel/gazebo_ros/scripts/spawn_model
    '''
    #rospy.init_node('spawn_ball')
    pass


# get ball location(x,y)
def get_ball_location(number=0):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        get_model_state = srv(model_name="soccer_ball", relative_entity_name='world')
        # rospy.loginfo("reset world")
    except rospy.ServiceExceptions as e:
        print("Service call failed %s"%e)

    return get_model_state.pose.position.x, get_model_state.pose.position.y


# reset gazebo world(it resets models)
def reset_world(robot_x=0,robot_y=0,robot_angle=0,ball_x=1,ball_y=0):
    rospy.wait_for_service('gazebo/reset_world')
    # ServiceProxy and call means `rosservice call /gazebo/reset_world`
    srv = rospy.ServiceProxy('gazebo/reset_world', std_srvs.srv.Empty)
    srv.call()

    # set the robot
    model_pose = Pose()
    #model_pose.position.x = robot_x
    model_pose.position.x = 3.0
    #model_pose.position.y = robot_y
    model_pose.position.y = robot_y
    model_pose.orientation.z = np.sin(robot_angle/2.0) 
    model_pose.orientation.w = np.cos(robot_angle/2.0) 
    modelstate = ModelState()
    modelstate.model_name = 'mobile_base'
    modelstate.reference_frame = 'world'
    modelstate.pose = model_pose
    set_model_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
    set_model_srv.call(modelstate)


    # set the ball
    model_pose = Pose()
    # model_pose.position.x = ball_x
    model_pose.position.x = 3.25
    # model_pose.position.y = ball_y
    model_pose.position.y = ball_y
    modelstate = ModelState()
    modelstate.model_name = 'soccer_ball'
    modelstate.reference_frame = 'world'
    modelstate.pose = model_pose
    set_model_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
    set_model_srv.call(modelstate)
    # rospy.loginfo("reset world")


# reset gazebo simulation(it resets models and **time**)
def reset_simulation(robot_x=0,robot_y=0,robot_angle=0,ball_x=1,ball_y=0):
    rospy.wait_for_service('/gazebo/reset_simulation')
    rospy.wait_for_service('/gazebo/set_model_state')
    # ServiceProxy and call means `rosservice call /gazebo/simulation_world`
    srv = rospy.ServiceProxy('/gazebo/reset_simulation', std_srvs.srv.Empty)
    srv.call()

    # set the robot
    model_pose = Pose()
    model_pose.position.x = robot_x
    model_pose.position.y = robot_y
    model_pose.orientation.z = np.sin(robot_angle/2.0) 
    model_pose.orientation.w = np.cos(robot_angle/2.0) 
    modelstate = ModelState()
    modelstate.model_name = 'mobile_base'
    modelstate.reference_frame = 'world'
    modelstate.pose = model_pose
    set_model_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
    set_model_srv.call(modelstate)


    # set the ball
    model_pose = Pose()
    model_pose.position.x = ball_x
    model_pose.position.y = ball_y
    modelstate = ModelState()
    modelstate.model_name = 'soccer_ball'
    modelstate.reference_frame = 'world'
    modelstate.pose = model_pose
    set_model_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
    set_model_srv.call(modelstate)
    rospy.loginfo("reset simulation")


# '''
# def spawn_turtlebot():
#     rospy.wait_for_service('/gazebo/spawn_urdf_model')
#     sp = SpawnModel
#     sp.model_name = 'turtlebot'
#     try:
#         srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', std_srvs.srv.Empty)
#         srv.call()
# '''
