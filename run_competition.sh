#!/bin/bash
# ==============================================================================
# F1TENTH Autonomous Racing - Competition Runner
# Course: Autonomous Vehicles
# Track: Circuit Zandvoort (Group 4)
# Driver: Alamgir Abu Hanif
# ==============================================================================

# Ensure script halts cleanly on Ctrl+C
trap 'echo -e "\n\033[1;33m[!] Stopping all nodes...\033[0m"; kill 0; exit' SIGINT SIGTERM

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: No ROS 2 installation found in /opt/ros!"
    exit 1
fi

# Source workspace install
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "Error: Workspace not built! Run 'colcon build' first."
    exit 1
fi

echo -e "\033[1;36m==============================================================\033[0m"
echo -e "\033[1;32m🏎️   F1TENTH AUTONOMOUS RACING COMPETITION RUNNER\033[0m"
echo -e "\033[1;37m    Group: 4 | Student: Alamgir Abu Hanif\033[0m"
echo -e "\033[1;37m    Track: Circuit Zandvoort\033[0m"
echo -e "\033[1;37m    Challenges: 10 Laps No Collision & Fastest Lap Time\033[0m"
echo -e "\033[1;36m==============================================================\033[0m"

echo -e "\n\033[1;34m[*] Launching F1TENTH Simulator & RViz2...\033[0m"
ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
SIM_PID=$!

# Give the simulator and RViz 3.5 seconds to initialize
sleep 3.5

echo -e "\n\033[1;32m[*] Starting Follow-the-Gap Controller with Lap Counter & Timer...\033[0m\n"
ros2 run controllers gap_node

# Wait for simulator if controller terminates
wait $SIM_PID
