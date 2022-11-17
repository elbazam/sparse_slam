#!/usr/bin/env python3

import roslaunch
import rospy

from std_srvs.srv import Empty
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

from room_slam.ros.respawn_model import  Respawn_model


import rospkg




class Repeat():

    def __init__(self):
        
        self.pub = rospy.Publisher('/Exp_index' , Int16 , queue_size=1)
        self.srv = rospy.Service('/SSLAM/restart' , Empty , self.request)
        self.finished_request_srv = rospy.Service('/vel_reset_srv',Empty,self.finished_request)

        rp = rospkg.RosPack()
        self.agent_path = rp.get_path('agent')
        self.object_localizer_path = rp.get_path('object_detector') 
        self.hector_slam_path = rp.get_path('hector_slam_launch') 
        
        
        self.count = [27,40,42,45,46,48]
        self.simulation_setup()
        self.configuration()
        
        
        self.idx = Int16()
        self.idx.data = self.count[0]
        
        
    def publishing(self):
        self.pub.publish(self.idx)

    def simulation_setup(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch_simulation = roslaunch.parent.ROSLaunchParent(self.uuid, [self.agent_path + "/launch/big_complex_world.launch"] )
        self.launch_simulation.start()
        
        rte = rospy.Rate(1/10)
        rte.sleep()
        print('Simulation uploaded successfully')
        
        self.rviz_simulation = roslaunch.parent.ROSLaunchParent(self.uuid, [self.object_localizer_path+ "/launch/gazebo_kinect.launch"])
        self.rviz_simulation.start()


        rte = rospy.Rate(1/20)
        rte.sleep()
        print('RVIZ initialization uploaded successfully')



    def configuration(self):
        
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch_sslam = roslaunch.parent.ROSLaunchParent(uuid, [self.object_localizer_path + "/launch/sslam_ln.launch"])
        self.launch_hector_slam = roslaunch.parent.ROSLaunchParent(uuid, [self.hector_slam_path + "/launch/hector_slam_demo.launch"])
        spawned = Respawn_model()
        while True:
            if spawned:
                break
            else:
                spawned = Respawn_model()
        self.launch_velocity = roslaunch.parent.ROSLaunchParent(uuid, [self.object_localizer_path + "/launch/run_vel_big_co.launch"])
        
        self.launch_sslam.start()
        self.launch_hector_slam.start()
        
        self.launch_velocity.start()
        print('-----------Starting simulation number: ', self.count[0])
        
    def request(self,srv):
        print('Recieved the request')
        self.launch_sslam.shutdown()
        self.launch_velocity.shutdown()
        self.launch_hector_slam.shutdown()
        
        rospy.sleep(1)
        
        self.count.pop(0) 
        try: self.idx.data = self.count[0]
        except: print('Finished')
        if not self.count:
            self.rviz_simulation.shutdown()
            self.launch_simulation.shutdown()
            rospy.signal_shutdown("Reached max iteration, finished with the code.")
        self.configuration()

    def finished_request(self , srv):
        print('Recieved the request')
        self.launch_sslam.shutdown()
        self.launch_velocity.shutdown()
        self.launch_hector_slam.shutdown()
        
        
        
        self.count.pop(0) 
        try: self.idx.data = self.count[0]
        except: print('Finished')
        if not self.count:
            rospy.sleep(5)
            self.rviz_simulation.shutdown()
            self.launch_simulation.shutdown()
            rospy.signal_shutdown("Reached max iteration, finished with the code.")
        self.configuration()

def msg():
    print('Finished.')
    vel = rospy.Publisher('/cmd_vel' , Twist , queue_size=1)
    for i in range(2):
        vel.publish(Twist())

if __name__ == '__main__':
    a = rospy.init_node('sslam_repeated', anonymous = True)
    R = Repeat()
    rte = rospy.Rate(1/10)
    while not rospy.is_shutdown():
        R.publishing()
        rte.sleep()
    rospy.on_shutdown(msg)
        