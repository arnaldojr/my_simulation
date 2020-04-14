
#!/bin/bash
echo ============================= RESOLVENDO CONFLITOS =============================
sudo apt-get install -y ros-melodic-industrial-core
sudo apt-get install -y ros-melodic-ros-controllers
sudo apt-get install -y ros-melodic-gazebo*
sudo apt-get install -y ros-melodic-robotis-*
sudo apt-get install -y ros-melodicc-gazebo-ros-control
sudo apt-get install -y ros-melodic-diff-drive-controller
sudo apt-get install -y ros-melodic-moveit*
sudo apt-get install -y ros-melodic-effort-controllers
sudo apt-get install -y ros-melodic-joint-state-controller
sudo apt-get install -y ros-melodic-position-controllers
sudo apt update -y
sudo apt upgrade -y
rm -rf ~/catkin_ws/src/turtlebot3_manipulation
cd ~/catkin_ws/src

echo ============================ CLONANDO REPOSITORIOS ============================

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git

cd ~/catkin_ws
catkin_make


echo excluindo os urdfs sem camera
mv $HOME/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/open_manipulator_x.urdf.xacro $HOME/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/open_manipulator.urdf.xacro
mv $HOME/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/open_manipulator_x.gazebo.xacro $HOME/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/open_manipulator.gazebo.xacro

cp $HOME/catkin_ws/src/my_simulation/models/garra/* $HOME/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description/urdf


echo ============================ SO TESTAR JOVEM! ============================

