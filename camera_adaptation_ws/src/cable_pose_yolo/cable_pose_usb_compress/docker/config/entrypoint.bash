#!/bin/bash

export ROS_DOMAIN_ID=87
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

sudo sysctl -w net.core.rmem_max=20971520
sudo sysctl -w net.core.wmem_max=20971520

source /opt/ros/humble/setup.bash

cd $CAMERA_ADAPTATION_WS_DIR
rm -rf ./build/ ./install/ ./log/
colcon build --packages-select cable_pose_yolo_compress --symlink-install --executor sequential --parallel-workers 1
source ./install/setup.bash

ros2 launch realsense2_camera rs_launch.py camera_namespace:=cam_3 camera_name:=camera enable_depth:=false rgb_camera.color_profile:=640x480x30 &

ros2 run tf2_ros static_transform_publisher 0 0 0.5 0 1 0 0 auxiliar_table camera_color_optical_frame &

ros2 launch cable_pose_yolo_compress cable_pose_yolo.launch.py