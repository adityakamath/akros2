#!/bin/bash
source /home/deck/.bashrc
source /opt/ros/humble/setup.bash
source /home/deck/SD512/ros2_ws/install/local_setup.bash

ros2 daemon stop
ros2 daemon start

ros2 launch akros2_bringup basestation_launch.py joy_config:=steamdeck viz_config:=foxglove
