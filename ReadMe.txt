___________________________________________________
_simulation gazebo
roslaunch mybot_gazebo mybot_world.launch

_controleur des moteurs:
roslaunch regul launcher_pc.launch

_____________________________________________________
Installation:

installer les packages :
(pour le controle des moteurs)
	ros-melodic-control 
(Pour le GPS)
	ros-melodic-hector-gazebo-plugins

Installation des modèles pour que gazebo les retrouvent
Placer dans le dossier ~/.gazebo/models le contenu du dossier model

Installation de la simu ROS:
dans un workspace ROS, placer le contenu du dossier ros dans le dossier src
un petit catkin_make et c'est bon



Pour allumer gazebo pour debug (le launcher mybot_world.launch lance déjà gazebo):
roscore & rosrun gazebo_ros gazebo

___________________________________
Pour modifier la mission, il faut modifier le fichier mission.cpp du package regul. La fonction read peut être modifiée pour soit lire un fichier (exemple data_traj.txt) soit suivre une suite d'objectif directement écrit dans le code (c'est le cas actuellement).
Pour lancer la simulation Gazebo :
roslaunch mybot_gazebo mybot_world.launch
Pour lancer le contrôleur du robot (il y a des modifications entre le code du robot réel et le code du robot simulé) :
roslaunch regul launcher_pc.launch





