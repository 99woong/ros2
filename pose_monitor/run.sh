#!/bin/bash

ros2 run vloc_receiver vloc_receiver_node &
sleep 1
ros2 run gls100_ros2 gls100_node &
sleep 1
ros2 run pose_monitor pose_monitor_node &


