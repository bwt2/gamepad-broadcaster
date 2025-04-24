#!/bin/bash
ROS_DISTRO="humble"

source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install
source install/setup.bash
ros2 run joy_udp_relay start