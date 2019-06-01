# bindRobot

## Acknowledgement

This work cannot be done without many open source projets. Special thanks to

- [moveit](https://github.com/ros-planning/moveit), used for motion planning
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [aubo_robot](https://github.com/ZhouYixuanRobtic/aubo_robot_realsense)

## Installation

### Install moveit

>rosdep update

>sudo apt-get update

>sudo apt-get dist-upgrade

>sudo apt-get install ros-kinetic-catkin python-catkin-tools

>sudo apt install ros-kinetic-moveit

>sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin

>sudo apt-get install ros-kinetic-moveit-visual-tools


### Aubo 网络连接

> 打开系统设置
> ->网络
> ->有线
> ->右下角的选项
> ->IPv4设置
> ->‘方法’选项中改为手动
> ->地址栏的‘增加’
> ->设置地址192.168.1.11（IP后面的11可以改为任意不同于ur5机械臂IP的值），子网掩码255.255.255.0，网关192.168.1.1，DNS服务器0.0.0.0(子网掩码、网关和DNS服务器设置为和机械臂的相同就行)
>
> 原文：https://blog.csdn.net/qq_37541593/article/details/81542153 

## Run with camera

### launch moveit

`roslaunch aubo_i5_moveit_config moveit_planning_execution.launch  sim:=false robot_ip:=192.168.1.101`

### launch rgbd camera

`roslaunch realsense2_camera rs_rgbd.launch enable_infra1:=false enable_infra2:=false`

### run bind node

`rosrun bindRobot camerattt`

