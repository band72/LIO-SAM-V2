#!/bin/bash
# ==============================================================================
# LIO-SAM ROS2 Bag Recording Script
# ==============================================================================
# This script records the essential LiDAR and IMU topics required to run 
# LIO-SAM later. It saves them into a designated output folder.
#
# Usage: ./record_survey.sh [output_bag_name]
# ==============================================================================

BAG_NAME=$1

if [ -z "$BAG_NAME" ]; then
    BAG_NAME="lio_sam_survey_$(date +%Y%m%d_%H%M%S)"
fi

echo "======================================================================"
echo " Starting LIO-SAM Data Recording"
echo "======================================================================"
echo "=> Saving bag to: $BAG_NAME"
echo "=> Recording topics: /points_raw, /imu_correct"
echo "=> Press Ctrl+C to stop recording when you are finished."
echo "----------------------------------------------------------------------"

source /opt/ros/jazzy/setup.bash

ros2 bag record -o "$BAG_NAME" \
    /points_raw \
    /imu_correct

echo "----------------------------------------------------------------------"
echo "=> Recording stopped."
