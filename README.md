# Oatmeal gazebo simulation

## Robot definition
Oatmeal in chinese is "小麦片". It is a two-wheeled balance robot that I made for my girlfriend.
Here oatmeal is the name of the entire project, and the project may contains multiple robots

## Build
```shell
cd oatmeal_ws/
catkin_make -j
```

## Using Gazebo
```shell
source /devel/setup.bash
roslaunch oatmeal_gazebo gazebo.launch
```

## Send msg/cmd from ros
```shell
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
axes: [1, 2, 3, 4, 5, 6]
buttons: [1, 2, 3, 4, 5, 6, 7, 8, 9]"
```

## View robot status data from ros
```shell
rostopic echo /carwheel/status
```

## Send msg/cmd from keyboard
```shell
cd oatmeal_ws/
source devel/setup.bash
rosrun oatmeal_gazebo ros_joy_keboard
```

## To quickly stop Gazebo if Ctrl-C is too slow
```shell
pkill -P $(pgrep -f gazebo.launch) ; pkill -9 gzserver ; pkill -9 gzclient
```
