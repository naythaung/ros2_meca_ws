#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source ~/meca_ws/install/setup.bash

BAG_PATH=~/meca_ws/phantom_session_1733   # <-- fill in your actual bag name

echo "Starting headless scene..."
ros2 launch meca500_scene_description headless_scene.launch.py > ~/meca_ws/scene.log 2>&1 &
SCENE_PID=$!

sleep 3

echo "Determining bag duration for randomized start point..."
DURATION=$(ros2 bag info "$BAG_PATH" 2>/dev/null | grep "Duration" | awk '{print $2}' | sed 's/s//')
OFFSET=$(awk -v max="$DURATION" 'BEGIN{srand(); print rand()*max}')
echo "Bag duration: ${DURATION}s - starting playback from offset ${OFFSET}s"

echo "Starting bag playback (looping)..."
ros2 bag play "$BAG_PATH" --topics /tf /image_raw --loop --start-offset "$OFFSET" > ~/meca_ws/bag.log 2>&1 &
BAG_PID=$!

sleep 2

echo "Generating ground truth (this will exit on its own when done)..."
ros2 run data_collection generate_mock_ground_truth

echo "Generation finished. Cleaning up..."
kill $BAG_PID $SCENE_PID 2>/dev/null || true
echo "Done. Check ~/meca_ws/mock_dataset/"