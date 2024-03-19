#!/bin/bash
if [ -z "$1" ]
  then
    echo "No argument supplied"
    exit 1
fi

cd /home/team2/dd2419_ws/bags && ros2 bag record /tf_static /scan /tf /motor/encoders -o ${1}