# Update Nov. 7,2016

# gazebo_utils API

## get_ball_location(number=0)
  
Get the ball location.
  
  
### Args:  
number(int) : ball number you want to know location(defalut=0)
  
  
### Returns:
x(float) : the location x  
y(float) : the location y  


## reset_world(robot_x=0,robot_y=0,robot_angle=0,ball_x=1,ball_y=0)

Reset simulation world.
It set any location of robots and balls you would like.
The simulation time isn't reset.
Any location mustn't be conflict.
  
  
### Args:  
robot_x(float) : the robot location x(defalut=0.0)  
robot_y(float) : the robot location y(default=0.0)  
robot_angle(float) : the robot angle. the unit is radian(defalut=0.0)  
ball_x(float) : the ball location x(defalut=1.0)  
ball_y(float) : the ball location y(defalut=(0.0)  
  
  
### Returns: void
  
  
## reset_simulation(robot_x=0,robot_y=0,robot_angle=0,ball_x=1,ball_y=0)
  
Reset simulation world.  
It set any location of robots and balls you would like.  
The simulation time is also reset.  
Any location mustn't be conflict.  
  
  
### Args:  
robot_x(float) : the robot location x(defalut=0.0)  
robot_y(float) : the robot location y(default=0.0)  
robot_angle(float) : the robot angle. the unit is radian(defalut=0.0)  
ball_x(float) : the ball location x(defalut=1.0)  
ball_y(float) : the ball location y(defalut=(0.0)  
  
  
### Returns: void


# gazebo_action API

## __init__(robot_name="mobile_base")

### Args:  
robot_name(string) : target robot name(defalut="mobile_base")


## __del__()

kill `gazebo` process(`gzclient` and `gzserver`)

### Args: void


## control_action(action)

Control a turtlebot action

### Args:  
action(int) : robot action  

   0: turtlebot doesn't move.  
   1: turtlebot moves fowrard  
   2:            backward  
   3: turtlebot rotates right  
   4:              left  


### Returns: void  


## get_image_array()

Get the image the turtlebot sees

### Args: void

### Returns:  
image_array(numpy array): [[b, g, r],[b, g, r] ...]
