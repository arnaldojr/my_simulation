# Interface de controle - Open Manipulator 

Este é um guia para  Interface de controle do Open Manipulator com Turtlebot3 Waffle


### Antes de qualquer coisa, vamos atualizar o repositório

Abra um terminal Crtl+Alt+t digite:

    roscd my_simulation
    
Atualize o Repositório

    git pull


### Agora vamos rodar os launchs necessários


Abra um novo terminal Crtl+Alt+t digite:

    roslaunch my_simulation sala404_circuito_gripper_ar_tracking.launch
    
Em um outro terminal Crtl+Shift+t:

    roslaunch turtlebot3_manipulation_moveit_config move_group.launch
    
Aperte o "play" no Gazebo para liberar a simulação

 ![aperte o play](https://github.com/arnaldojr/my_simulation/blob/master/garra/play.png)
 
    
Em um outro terminal Crtl+Shift+t

    roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
    
  
 Em um outro terminal Crtl+Shift+t, rode o programa exemplo:

    rosrun my_simulation open_manipulator.py 
    
    
 

    
