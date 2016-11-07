## SUSANoh



## Notice by ueno

If you need friction ball,execute this command.

```
export GAZEBO_MODEL_PATH=`pwd`/worlds/models/
```

and launch simulator.
```
cd worlds
roslaunch test.launch dir:=`pwd`
```

### change a robot initial pose
If you want to change each robot location,change a arg `init_pose` in launch file.

For example, a robot locates (x,y,z) = (1,2,3) and (roll, pitch, yaw) = (-1, -2, -3)

`<arg name="init_pose" value="-x 1 -y 2 -z 3 -R -1 -P -2 -Y -3"/>`



- [] make LRF Rulebase
   - [] catch LRF value

- [C] catch camera data

- [C] add ball friction in world file  
   - [C] confirm friction of ball in gazebo-7  
**caution**
ball friction is not supported in any gazebo. \<velocity_decay\> tag realize similar things.
But this tag applicates a model in the air. 

- [C] control multiple robots


## Notice by Osawa

emacs /home/osawa/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/turtlebot/turtlebot_description/urdf/turtlebot_gazebo.urdf.xacro

width and height 
480 -> 60