#!/bin/bash
# Save map using Nav2 map_saver (ROS 2)
# Usage: ./savemap.sh [map_name]
# Default map name: mymap

MAP_NAME="${1:-mymap}"
MAP_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Saving map as: ${MAP_DIR}/${MAP_NAME}"
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" --ros-args -p save_map_timeout:=10000

if [ $? -eq 0 ]; then
    echo "Map saved successfully: ${MAP_DIR}/${MAP_NAME}.yaml and ${MAP_DIR}/${MAP_NAME}.pgm"
else
    echo "Error: Failed to save map. Make sure SLAM or map_server is running."
fi
