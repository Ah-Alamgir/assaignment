#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/abuhanif/F1Tenth-Repository/install/setup.bash

echo "Starting simulator..."
ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
SIM_PID=$!

sleep 4

echo "Starting Follow-the-Gap Controller..."
ros2 run controllers gap_node &
GAP_PID=$!

sleep 15

echo "Stopping test..."
kill $GAP_PID 2>/dev/null
kill $SIM_PID 2>/dev/null
wait $SIM_PID 2>/dev/null
echo "Test complete!"
