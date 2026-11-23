#!/bin/bash

export ROS_DOMAIN_ID=87
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/cyclone_dds_config.xml

sudo sysctl -w net.core.rmem_max=20971520
sudo sysctl -w net.core.wmem_max=20971520

source /opt/ros/humble/setup.bash
ros2 daemon stop

cd $CAMERA_ADAPTATION_WS_DIR
rm -rf ./build/ ./install/ ./log/
colcon build --packages-select cable_pose_offload --symlink-install --executor sequential --parallel-workers 1
source ./install/setup.bash

ros2 run tf2_ros static_transform_publisher 0 0 0.5 0 1 0 0 auxiliar_table camera_color_optical_frame &

exec ros2 launch cable_pose_offload cable_pose_yolo.launch.py