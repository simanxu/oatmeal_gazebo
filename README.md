# Oatmeal gazebo simulation

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

## To quickly stop Gazebo if Ctrl-C is too slow
```shell
pkill -P $(pgrep -f gazebo.launch) ; pkill -9 gzserver ; pkill -9 gzclient
```
