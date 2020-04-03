
#!/bin/bash
echo ============================= RESOLVENDO CONFLITOS =============================

sudo apt-get install ros-melodic-ros-controllers
sudo apt-get install ros-melodic-gazebo*
sudo apt-get install  ros-melodic-robotis-*
sudo apt-get install ros-melodicc-gazebo-ros-control
sudo apt-get install ros-melodic-diff-drive-controller
sudo apt-get install ros-melodic-moveit
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-position-controllers
sudo apt update
rm -rf ~/catkin_ws/src/turtlebot3_manipulation
cd ~/catkin_ws/src

echo ============================ CLONANDO REPOSITORIOS ============================
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
cd turtlebot3_manipulation_simulations
git checkout melodic-devel
cd ~/catkin_ws
sudo apt install ros-melodic-moveit 
catkin_make

echo ============================ SO TESTAR JOVEM! ============================
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation.launch 
