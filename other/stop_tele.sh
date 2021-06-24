#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/warthog/warthog/devel/setup.bash
rosnode kill --a
sleep 1
sudo service warthog_tele_start stop
sleep 1
sudo service warthog_tele_stop stop
