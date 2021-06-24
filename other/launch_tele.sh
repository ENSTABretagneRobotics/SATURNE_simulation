#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/warthog/warthog/devel/setup.bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
roslaunch velocity_control telecommande.launch
