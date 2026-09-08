#!/usr/bin/env bash

cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit
source /opt/ros/lyrical/setup.bash || exit
colcon build || exit
source install/setup.bash || exit
ros2 launch meca500_moveit_config move_group.launch.py
