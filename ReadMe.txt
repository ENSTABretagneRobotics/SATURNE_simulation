___________________________________________________
_simulation gazebo
roslaunch mybot_gazebo mybot_world.launch

_controleur des moteurs:
roslaunch regul launcher_pc.launch

_____________________________________________________
Installation:

installer les packages :
(pour le controle des moteurs)
	ros-melodic-velocity-controllers
(Pour le GPS)
	ros-melodic-hector-gazebo-plugins

Installation des modèles pour que gazebo les retrouvent
Placer dans le dossier ~/.gazebo/models le contenu du dossier model

Installation de la simu ROS:
Placer dans le dossier ~/catkin_ws/src le contenu du dossier src (en supposant que catkin_ws est le workspace ROS)
cd ~/catkin_ws
catkin_make
source devel/setup.bash

Pour allumer gazebo pour debug (le launcher mybot_world.launch lance déjà gazebo):
roscore & rosrun gazebo_ros gazebo

___________________________________
Pour modifier la mission, il faut modifier le fichier mission.cpp du package regul. La fonction read peut être modifiée pour soit lire un fichier (exemple data_traj.txt) soit suivre une suite d'objectif directement écrit dans le code (c'est le cas actuellement).
Pour lancer la simulation Gazebo :
# For VMware : export SVGA_VGPU10=0
roslaunch mybot_gazebo mybot_world.launch
Pour lancer le contrôleur du robot régulant la position d'une remorque tractée à une distance supposée connue (voir https://www.ensta-bretagne.fr/jaulin/magmap.html, il y a des modifications entre le code du robot réel et le code du robot simulé) :
roslaunch regul launcher_pc.launch
