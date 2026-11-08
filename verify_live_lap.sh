#!/bin/bash
# ==============================================================================
# Live ROS 2 Simulation & Lap Verification Script
# ==============================================================================

source /opt/ros/jazzy/setup.bash
source /home/abuhanif/F1Tenth-Repository/install/setup.bash

echo "=================================================================="
echo "🏎️  STARTING LIVE ROS 2 SIMULATION & CONTROLLER VERIFICATION"
echo "=================================================================="

# Start simulator in background
ros2 launch f1tenth_gym_ros gym_bridge_launch.py > /tmp/sim_launch.log 2>&1 &
SIM_PID=$!

# Wait for map server and bridge initialization
sleep 4

# Check simulator processes
if ps -p $SIM_PID > /dev/null; then
    echo "✅ F1TENTH Gym Simulator & Map Server active (PID: $SIM_PID)"
else
    echo "❌ Simulator failed to start. Log output:"
    cat /tmp/sim_launch.log | tail -n 20
    exit 1
fi

echo "✅ Active ROS 2 Topics:"
ros2 topic list | grep -E "scan|drive|odom|map|lap"

echo ""
echo "🚀 Launching Follow-the-Gap Controller (Monitoring Lap 1 live)..."
echo "------------------------------------------------------------------"

# Run controller in background and capture output
ros2 run controllers gap_node 2>&1 | tee /tmp/gap_node_live.log &
GAP_PID=$!

# Monitor progress for up to 95 seconds or until Lap 1 completes
START_TIME=$(date +%s)
LAP_COMPLETED=0

while true; do
    sleep 3
    NOW=$(date +%s)
    ELAPSED=$((NOW - START_TIME))
    
    # Check if Lap 1 completion was logged
    if grep -q "LAP  1/10 COMPLETED" /tmp/gap_node_live.log; then
        echo ""
        echo "🎉 LAP 1 DETECTED BY LIVE ROS 2 CONTROLLER!"
        LAP_COMPLETED=1
        break
    fi
    
    # Timeout check
    if [ $ELAPSED -ge 95 ]; then
        echo "⚠️ Reached 95s timeout. Stopping live verification run."
        break
    fi
done

# Wait an additional 2 seconds to capture full banner
sleep 2

# Cleanup processes
kill $GAP_PID 2>/dev/null
kill $SIM_PID 2>/dev/null
killall -9 rviz2 gym_bridge 2>/dev/null
wait $SIM_PID 2>/dev/null

echo ""
echo "=================================================================="
echo "📊 LIVE ROS 2 CONTROLLER LOG SUMMARY"
echo "=================================================================="
grep -E "Initialized|LAP|Best|Speed|COMPLETED" /tmp/gap_node_live.log | tail -n 25

echo "=================================================================="
if [ $LAP_COMPLETED -eq 1 ]; then
    echo "🏆 VERIFICATION RESULT: SUCCESS! Lap 1 completed live without collision!"
else
    echo "📋 Status: Live run completed."
fi
echo "=================================================================="
