
import rospy

from gazebo_ros import gazebo_interface

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