#!/usr/bin/env python3

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose , Twist



    
def Respawn_model(name = 'agent' , frame = 'world'):
    '''Respawn gazebo model.
    Upon succession the function return the bool True, else return False'''
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    try:
        gazebo_model_srv = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        model_vel = Twist()
        model_pose = Pose()
        
        model = ModelState()
        model.model_name = name
        model.twist = model_vel
        model.pose = model_pose
        model.reference_frame = frame
        gazebo_model_srv(model)
        print('Model Respawned')
        return True

    except:
        print("Service call failed, could not respawn model")
        return False
