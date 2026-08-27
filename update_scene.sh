cd ~/meca_ws
colcon build --packages-select meca500_scene_description
source install/setup.bash
ros2 launch meca500_scene_description view_scene.launch.py