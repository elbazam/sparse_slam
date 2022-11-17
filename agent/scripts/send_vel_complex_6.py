#!/usr/bin/env python3

import rospy


import numpy as np
# import threading
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates



class Velocity():

    def __init__(self):

        
        self.locations = np.array([[7.6,0],[9,2],[9,3.7],[5.3,4],[5,0.8],[0,-0.5]])
        self.stop = len(self.locations)
        self.next_index = 0

        self.x = 0
        self.y = 0
        

        self.e = 0
        self.eth = 0

        self.C_P = 1
        self.C_D = 0.4

        self.V = Twist()
        self.Pub = rospy.Publisher('/cmd_vel' , Twist , queue_size=1)
        
        self.topic = '/gazebo/model_states'
        self.model_name = 'agent'

        self.msg = rospy.wait_for_message(self.topic , ModelStates)

        try: self.model_index = self.msg.name.index(self.model_name)
        except: print('No model with the name of ' + self.model_name)
        rospy.Timer(rospy.Duration(0.1), self.callback)
        rospy.Subscriber(self.topic,ModelStates, self.real_pose_callback)


    def real_pose_callback(self,data):
        self.msg = data
        
        

    def callback(self,t):
        
        
        self.x = self.msg.pose[self.model_index].position.x
        self.y = self.msg.pose[self.model_index].position.y
        
        qu = self.msg.pose[self.model_index].orientation
        r = R.from_quat([qu.x, qu.y, qu.z, qu.w])
        self.th = r.as_euler('zxy')[0]
       
        
        if self.next_index != self.stop:
            xg = self.locations[self.next_index,0]
            yg = self.locations[self.next_index,1]
            theta = np.arctan2(yg - self.y , xg - self.x)
            
            
            err = self.C_P * np.linalg.norm([yg - self.y , xg - self.x]) - self.C_D * self.e
            
            th_err = self.C_P * (theta - self.th) - self.C_D * self.eth
            if np.abs(theta - self.th) > np.pi: th_err = -th_err
            
            self.eth = th_err
            if np.abs(theta - self.th) > np.pi/4:
                err = 0.01
            self.e = err
            self.V.linear.x = np.min([0.07 , err])
            
            temp = np.min([0.07 , np.abs(th_err)])
            self.V.angular.z = np.sign(th_err) * temp
            if np.abs(th_err) - 0.05 < 0.2: self.eth = 0

            self.Pub.publish(self.V)
            if np.linalg.norm([yg - self.y , xg - self.x]) < 0.3:
                self.next_index += 1
                if self.next_index != self.stop: print('next destination: ' , self.locations[self.next_index,:])
                self.V = Twist()
                self.Pub.publish(self.V)

        else:
            rospy.wait_for_service('/vel_reset_srv')
            rospy.ServiceProxy('/vel_reset_srv',Empty)()
            rospy.sleep(2)
        



if __name__ == '__main__':

    rospy.init_node('velocity_node' , anonymous=True)
    rospy.sleep(2)
    Vel = Velocity()
    rospy.spin()
    for i in range(10):
        Vel.Pub.publish(Twist())
    