#!/bin/bash
# ==============================================================================
# LIO-SAM Survey Mapping Runner
# ==============================================================================
# This script automates the process of launching LIO-SAM, playing a ROS2
# bag file, and optionally extracting the final PCD map.
#
# Usage: ./run_survey.sh [path_to_bag_file]
# ==============================================================================

# Default bag file if none is provided
BAG_FILE=${1:-"/home/artwalk/Downloads/campus_small_dataset_ros2"}
MAP_DEST="/home/artwalk/Downloads/LIO_SAM_MAP/"

echo "======================================================================"
echo " Starting LIO-SAM High-Precision Survey Run"
echo "======================================================================"

# 1. Source ROS2 and workspace
echo "=> Sourcing ROS2 and local workspace..."
source /opt/ros/jazzy/setup.bash
source /home/artwalk/ros2_ws/install/setup.bash

# 2. Launch LIO-SAM in the background
echo "=> Launching LIO-SAM..."
ros2 launch lio_sam run.launch.py &
LIO_PID=$!

# 3. Wait for nodes to initialize
echo "=> Waiting 5 seconds for LIO-SAM to initialize RViz and mapping nodes..."
sleep 5

# 4. Play the bag file
echo "=> Playing dataset: $BAG_FILE"
echo "----------------------------------------------------------------------"
ros2 bag play "$BAG_FILE" --clock

# 5. Wait for backlog to finish processing before saving
echo "----------------------------------------------------------------------"
echo "=> Bag playback finished!"
echo "   Because you are using high-precision settings, LIO-SAM may still be"
echo "   processing the backlog of frames in the background."
echo ""
echo "   WAIT until you see the loop closures in the terminal output, or until"
echo "   the map stops updating in RViz."
echo ""
read -p "   Do you want to save the final map as a PCD file? [y/N]: " SAVE_MAP

if [[ "$SAVE_MAP" =~ ^[Yy]$ ]]; then
    # 6. Save the Map
    echo "=> Saving Map to: $MAP_DEST"
    ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.0, destination: '$MAP_DEST'}"
    echo "=> Map saved!"
else
    echo "=> Skipping PCD map save."
fi

# 7. Cleanup
echo "----------------------------------------------------------------------"
echo "=> Press Ctrl+C to stop LIO-SAM and exit this script completely."
wait $LIO_PID
