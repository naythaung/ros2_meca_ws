#!/usr/bin/env bash

cd ~/Documents/ros2_meca_ws || exit 1

source /opt/ros/lyrical/setup.bash
source install/setup.bash

ros2 launch meca500_tasks move_to_pose.launch.py
