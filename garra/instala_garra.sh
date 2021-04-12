#!/bin/bash
sudo apt update
echo ============================= RESOLVENDO CONFLITOS =============================
sudo apt-get install -y ros-noetic-industrial-core
sudo apt-get install -y ros-noetic-ros-controllers
sudo apt-get install -y ros-noetic-gazebo*
sudo apt-get install -y ros-noetic-robotis-*
sudo apt-get install -y ros-noeticc-gazebo-ros-control
sudo apt-get install -y ros-noetic-diff-drive-controller
sudo apt-get install -y ros-noetic-moveit*
sudo apt-get install -y ros-noetic-effort-controllers
sudo apt-get install -y ros-noetic-joint-state-controller
sudo apt-get install -y ros-noetic-position-controllers
cd ~/catkin_ws/src

echo ============================ CLONANDO REPOSITORIOS ============================

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
git clone  https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins 


cd ~/catkin_ws
catkin_make
