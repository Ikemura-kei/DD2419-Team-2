#!/bin/bash
if [ -z "$1" ]
  then
    echo "No argument supplied"
    exit 1
fi

cd ./bags && \
    ros2 bag record /tf_static /scan /tf /extracted_lines /extracted_points /ekf_slam_traj /ekf_slam_log -o ${1}