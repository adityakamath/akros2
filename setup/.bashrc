# ros2 environment variables
export ROS_DISTRO=humble
export ROS_VERSION=2
source /opt/ros/$ROS_DISTRO/setup.bash

# ros2 source and build
alias rosws='cd ~/ros2_ws'
alias sls='source ~/ros2_ws/install/local_setup.bash'
alias build_all='MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential'
alias build_stats='MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential --cmake-args -DFASTDDS_STATISTICS=ON'
alias build_only='MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential --packages-select'
alias build_only_stats='MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential --cmake-args -DFASTDDS_STATISTICS=ON --packages-select'
alias build_resume='MAKEFLAGS=¨-j1 -l1" colcon build --symlink-install --executor sequential --packages-skip-build-finished'
alias dep_install='rosdep install --from-paths src --ignore-src -r -y'

# ros2 tools
alias fgb='srs && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'

# AKROS2 launch
alias bringup='rosws && sls && ros2 launch akros2_bringup bringup_launch.py laser:=false joy_config:=none control:=false viz_config:=none'
alias bringup_local='rosws && sls && ros2 launch akros2_bringup bringup_launch.py laser:=false control:=false joy_config:=sn30pro viz_config:=foxglove'
alias basestation='rosws && sls && ros2 launch akros2_bringup basestation_launch.py joy_config:=sn30pro viz_config:=foxglove'
alias control='rosws && sls && ros2 launch akros2_base control_launch.py'
alias viz='rosws && sls && ros2 launch akros2_bringup viz_launch.py viz_config:=foxglove'
alias unity='rosws && sls && ros2 launch ros_tcp_endpoint endpoint.py'