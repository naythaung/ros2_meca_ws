#!/bin/bash
set -e

echo "Checking prerequisites..."

# Check Windows camera server is reachable
GATEWAY_IP=$(ip route show | grep default | awk '{print $3}')
if ! curl -s -o /dev/null -m 2 "http://${GATEWAY_IP}:5000/"; then
    echo "❌ Camera server not reachable at http://${GATEWAY_IP}:5000/"
    echo "   -> Start camera_server.py on Windows first."
    exit 1
fi
echo "✅ Camera server reachable."

# Check Meca500 is reachable
ROBOT_IP="192.168.0.100"
if ! ping -c 1 -W 2 "$ROBOT_IP" > /dev/null; then
    echo "❌ Meca500 not reachable at $ROBOT_IP"
    echo "   -> Check power and Ethernet connection."
    exit 1
fi
echo "✅ Meca500 reachable."

echo "All checks passed. Launching..."
source /opt/ros/jazzy/setup.bash
source ~/meca_ws/install/setup.bash
ros2 launch meca500_driver full_system.launch.py