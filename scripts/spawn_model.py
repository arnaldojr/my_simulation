
#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel 
from geometry_msgs.msg import Pose
import rospy, tf 
import random

poses = [(-2.30 , 3.69  ,0 ), (-5.80 ,3.69 ,0), (-3.45, -2.88 ,0), (-4.30, -1.94 ,0), ( -2.60 ,0.49, 0) , (5.49 ,1.66 ,0), (-1.26, -0.66 ,0), (2.75 ,2.74 ,0) , (1.71, -2.45 ,0) ]
creepers = ["creeper_11_azul", "creeper_11_laranja", "creeper_12_azul","creeper_13_verde", "creeper_21_verde", "creeper_22_azul", "creeper_23_verde", "creeper_51_azul", "creeper_52_verde" ]


for x in range(9):
    creeper=creepers[x]
    print("deletando o creeper ",creeper)
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    delete_model(creeper) 


for y in range(9):
    positions=random.sample(poses,1)
    pos=positions[0]
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]


    get_creeper=random.sample(creepers,1)
    creeper=get_creeper[0]
   
    pwd = '/home/borg/catkin_ws/src/my_simulation/models/forca/creepers/{}/model.sdf'.format(creeper)

    print ("lista de creepers disponiveis: ",creepers)
    print ("posicoes disponiveis no mapa: ",poses)
    print("=====================================================================================================================")
    print("creeper escolhido:",creeper)
    print("posição no mapa: ",pose)

    creeper_clone = creeper + "_clone"    
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
        model_name=creeper_clone,
        model_xml=open(pwd, 'r').read(),
        robot_namespace='/mybot',
        initial_pose=pose,
        reference_frame='world'
    )

    creepers.remove(creeper) 
    poses.remove(pos)

