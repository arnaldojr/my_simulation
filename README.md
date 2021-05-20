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


## turtlebot3 gazebo Alvar

Vamos executar o Alvar:

Abra um terminal Crtl+Alt+t digite:

    cd catkin_ws/src

Abra o mundo do Gazebo com os marcadores:

     roslaunch my_simulation mundoteste_ar_tracking.launch  

*Abra um novo terminal* Crtl+Alt+t e abra a câmera:

     rqt_image_view  

*Abra um novo terminal* Crtl+Alt+t e abra o RViz:

     roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch 


*No Rviz* vamos fazer algumas configurações para visualizar o Alvar:

No menu lateral esquerdo do Rviz:

    1. A primeira opção *fixed Frame*: Troque de odom para *camera_link*.
    2. A opção *TF* está desmarcada.: Marque está opção
 
 *Abra um novo terminal* Crtl+Alt+t e visualize o topico ar_pose_marker:

     rostopic echo /ar_pose_marker


# Videos para trabalhar com o Gazebo

Abaixo temos duas playlists com videos que ajudam na utilização do simulador Gazebo

https://www.youtube.com/watch?v=Nay-DI0EqzI&list=PLVU3UhXa4-X_XYVuaVmQkJdzWCxmREhrK

https://www.youtube.com/watch?v=X9Sp60IPk0M&list=PLM8rZg4fCalht-rexa91MO1y3jxm9mwOa



# Link original das imagens

cavalo: https://www.flickr.com/photos/33598193@N03/3705232382

Vaca:  https://www.vetsmart.com.br/be/raca/17010/holandesa

Cachorro: https://www.flickr.com/photos/150258205@N05/32255598460