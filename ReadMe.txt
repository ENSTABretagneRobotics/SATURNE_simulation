Installation :

Assuming that you already have ROS, install those packages (replace noetic by e.g. melodic for Ubuntu 18.04, see https://www.ensta-bretagne.fr/lebars/tutorials/ros_noetic_vs2019.txt for Windows) :
(for motor control)
	build-essential ros-noetic-robot ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
(for compass)
	ros-noetic-gazebo-plugins
(for GPS)
	ros-noetic-hector-gazebo-plugins

Place the content of the models folder in ~/.gazebo/models so that Gazebo finds the models.

Place the content of the src folder in ~/catkin_ws/src (assuming catkin_ws is the ROS workspace), then :
cd ~/catkin_ws
catkin_make
source devel/setup.bash
___________________________________
Usage :

To modify the mission, change mission.cpp from regul package. read() function can be modified to either read a file (e.g. data_traj.txt) or follow a sequence of "objectifs" directly written in the code (it is the default).

To launch Gazebo simulation :
# For VMware virtual machines : export SVGA_VGPU10=0
source devel/setup.bash
roslaunch mybot_gazebo mybot_world.launch

To launch the robot controller assuming we control the position of a trailer attached at a known distance (see https://www.ensta-bretagne.fr/jaulin/magmap.html, there are changes between the real and simulated robot...) :
source devel/setup.bash
roslaunch regul launcher.launch
