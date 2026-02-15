#!/bin/bash
# Save Cartographer map (ROS 2)
# Usage: ./carto_savemap.sh [map_name]
# Default map name: mymap

MAP_NAME="${1:-mymap}"
MAP_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Finishing Cartographer trajectory..."

# Finish the current trajectory
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
sleep 1

# Write the serialized state (pbstream)
echo "Writing pbstream to: ${MAP_DIR}/${MAP_NAME}.pbstream"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${MAP_DIR}/${MAP_NAME}.pbstream'}"
sleep 1

# Convert pbstream to map files (pgm + yaml)
echo "Converting pbstream to map..."
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename="${MAP_DIR}/${MAP_NAME}.pbstream" \
    -map_filestem="${MAP_DIR}/${MAP_NAME}"

if [ $? -eq 0 ]; then
    echo "Cartographer map saved successfully:"
    echo "  ${MAP_DIR}/${MAP_NAME}.pbstream"
    echo "  ${MAP_DIR}/${MAP_NAME}.yaml"
    echo "  ${MAP_DIR}/${MAP_NAME}.pgm"
else
    echo "Error: Failed to convert pbstream. Make sure cartographer_ros is installed."
fi