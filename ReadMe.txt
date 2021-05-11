Installation:

Assuming that you already have ROS, install those packages (replace noetic by e.g. melodic for Ubuntu 18.04):
(for motor control)
	build-essential ros-noetic-robot ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
(for compass)
	ros-noetic-gazebo-plugins
(for GPS)
	ros-noetic-hector-gazebo-plugins

Place the content of the models folder in ~/.gazebo/models so that Gazebo finds the models.

Place the content of the src folder in ~/catkin_ws/src (assuming catkin_ws is the ROS workspace), then:
cd ~/catkin_ws
catkin_make
source devel/setup.bash

(Pour allumer gazebo pour debug (le launcher mybot_world.launch lance déjà gazebo):
roscore & rosrun gazebo_ros gazebo)
___________________________________
Usage:

Pour modifier la mission, il faut modifier le fichier mission.cpp du package regul. La fonction read peut être modifiée pour soit lire un fichier (exemple data_traj.txt) soit suivre une suite d'objectifs directement écrit dans le code (c'est le cas actuellement).

To launch Gazebo simulation:
# For VMware virtual machines: export SVGA_VGPU10=0
source devel/setup.bash
roslaunch mybot_gazebo mybot_world.launch

To launch the robot controller assuming we control the position of a trailer attached at a known distance (see https://www.ensta-bretagne.fr/jaulin/magmap.html, il y a des modifications entre le code du robot réel et le code du robot simulé):
source devel/setup.bash
roslaunch regul launcher_pc.launch
