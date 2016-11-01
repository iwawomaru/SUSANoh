## SUSANoh



## Notice by ueno

If you need friction ball,execute this command.

```
export $GAZEBO_MODEL_PATH=`pwd`/worlds/models/
```

and launch simulator.
```
cd worlds
roslaunch test.launch dir:=`pwd`
```

[] solve the problem that the image array isn't sometimes caught

[] make LRF Rulebase
    [] catch LRF value


[] install Gzweb environment in server

[] make ROS package to launch environment
    [] catch model file from this repository


[C] catch camera data

[C] add ball friction in world file  
    [C] confirm friction of ball in gazebo-7
**caution**
ball friction is not supported in any gazebo. <velocity_decay> tag realize similar things.
But this tag applicates a model in the air. 