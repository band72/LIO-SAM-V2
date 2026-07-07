#!/bin/bash
# ==============================================================================
# LIO-SAM Survey Mapping Runner
# ==============================================================================
# This script automates the process of launching LIO-SAM and playing a ROS2
# bag file for offline, high-precision surveying.
#
# Usage: ./run_survey.sh [path_to_bag_file]
# Example: ./run_survey.sh /home/artwalk/Downloads/campus_small_dataset_ros2
# ==============================================================================

# Default bag file if none is provided
BAG_FILE=${1:-"/home/artwalk/Downloads/campus_small_dataset_ros2"}

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
echo "   NOTE: Using --clock to provide simulated time to LIO-SAM."
echo "   If the high-precision settings cause LIO-SAM to fall behind,"
echo "   you can edit this script and add '--rate 0.5' to run at half speed."
echo "----------------------------------------------------------------------"
ros2 bag play "$BAG_FILE" --clock

# 5. Cleanup when the bag finishes or the user presses Ctrl+C
echo "----------------------------------------------------------------------"
echo "=> Bag playback finished!"
echo "   LIO-SAM is likely still processing the final backlog of frames."
echo "   You can continue to explore the map in RViz."
echo "   Press Ctrl+C to stop LIO-SAM and exit this script completely."
wait $LIO_PID
