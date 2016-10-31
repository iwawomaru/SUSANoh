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


[C] catch camera data

[] make LRF Rulebase
    [] catch LRF value

[C] add ball friction in world file  
    [C] confirm friction of ball in gazebo-7
**caution**
ball friction is not supported in any gazebo. <velocity_decay> tag realize similar things.
But this tag applicates a model in the air. 

[] install Gzweb environment in server

[] make ROS package to launch environment
    [] catch model file from this repository 
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
