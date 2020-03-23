# turtlebot3 gazebo insper

Mapas do simulador Gazebo para o robo turtlebot3 do laboratorio de informatica, lab 404, para a materia de robotica computacional, do curso de engenharia de computação Insper. 

Para rodar faça:

Abra um terminal Crtl+Alt+t digite:

    cd catkin_ws/src

Clone este repositorio dentro da pasta catkin_ws/src.

    git clone https://github.com/arnaldojr/my_simulation.git  

Execute o comando catkin_make na pasta raiz catkin_ws
    
    cd ..
    catkin_make
    
Execute no terminal o comando abaixo para adionar o path do gazebo no .bashrc.

    echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/my_simulation/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
    source ~/.bashrc

Para executar rode o roslauch:

    roslaunch my_simulation mundoteste.launch



