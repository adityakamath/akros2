#!/bin/bash
source /home/deck/.bashrc
source /opt/ros/humble/setup.bash
source /home/deck/SD512/ros2_ws/install/local_setup.bash

rviz2 -d /home/deck/SD512/ros2_ws/src/akros2_bringup/viz/default.rviz
