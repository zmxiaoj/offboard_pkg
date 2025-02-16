#!/bin/bash

# Default values
RECORD_PATH="/root/catkin_ws/data"
PREFIX="flight"
RIGID_BODY="uav"

# Create bag directory if not exists
mkdir -p ${RECORD_PATH}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --path)
            RECORD_PATH="$2"
            shift 2
            ;;
        --prefix)
            PREFIX="$2"
            shift 2
            ;;
        --rigid-body)
            RIGID_BODY="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# Get current timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FILENAME="${RECORD_PATH}/${PREFIX}_${TIMESTAMP}"

# Print recording info
echo "Recording flight data to: ${FILENAME}.bag"
echo "Using rigid body name: ${RIGID_BODY}"

# Start recording
rosbag record -O ${FILENAME} \
    /mavros/state \
    /mavros/imu/data \
    /mavros/local_position/pose \
    /mavros/setpoint_position/local \
    /camera/infra1/image_rect_raw \
    /camera/infra2/image_rect_raw \
    /camera/depth/image_rect_raw \
    /camera/color/image_raw \
    /camera/imu \
    /visualization/current_pose \
    /visualization/setpoint_pose \
    /vrpn_client_node/${RIGID_BODY}/pose \
    /trajectory_cmd \
    /land_cmd \
    /livox/imu \
    /livox/lidar \
    /Odometry
