#!/bin/bash

# Navigate to the ROS 2 workspace and source it
cd ~/ros2_ws
source install/setup.bash

# Launch each command in a new terminal tab with a delay between each

# Set up the CAN interface
#gnome-terminal --tab -- bash -c "sudo ip link set can0 up type can bitrate 500000; exec bash"
#sleep 2

# Start CAN data dump
#gnome-terminal --tab -- bash -c "candump can0; exec bash"
#sleep 2

# Launch ROS nodes in sequence
gnome-terminal --tab -- bash -c "ros2 launch scout_base scout_mini_base.launch.py; exec bash"
sleep 5

gnome-terminal --tab -- bash -c "ros2 launch ouster_ros sensor.launch.xml    \
    sensor_hostname:=os-122405000193.local; exec bash" 
sleep 5

gnome-terminal --tab -- bash -c "ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true depth_module.profile:=640x480x15; exec bash"
sleep 5

# Change ownership of the USB device before running the vectornav node
gnome-terminal --tab -- bash -c "sudo chown imad /dev/ttyUSB0 && ros2 launch vectornav vectornav.launch.py || echo 'Failed to run vectornav'; exec bash"
sleep 5

# Uncomment and use if you have a custom transform script
# gnome-terminal --tab -- bash -c "python3 transform.py; exec bash"
# sleep 5

