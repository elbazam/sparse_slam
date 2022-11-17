#!/usr/bin/env python3

import rospy
import numpy as np
import pandas


from std_msgs.msg import Int16
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.msg import ModelStates

from scipy.spatial.transform import Rotation

def get_Pose(pose):
    pose_temp = Pose()
    pose_temp.position.x = pose[0]
    pose_temp.position.y = pose[1]
    pose_temp.position.z = 0
    r = Rotation.from_euler('z',pose[2], degrees=False)
    pose_temp.orientation.x = r.as_quat()[0]
    pose_temp.orientation.y = r.as_quat()[1]
    pose_temp.orientation.z = r.as_quat()[2]
    pose_temp.orientation.w = r.as_quat()[3]
    return pose_temp

def update_path(path, new_pose, update_th, frame_id):
    if len(path.poses) == 0:
        pose_stamp = PoseStamped()
        pose_stamp.header.frame_id = frame_id
        pose_stamp.header.stamp = rospy.Time.now()
        pose_stamp.pose = get_Pose(new_pose)
        path.poses.append(pose_stamp)
    else:
        last_pose = np.array([path.poses[-1].pose.position.x, path.poses[-1].pose.position.y])
        if np.linalg.norm(new_pose[:2] - last_pose) > update_th:
            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id = frame_id
            pose_stamp.header.stamp = rospy.Time.now()
            pose_stamp.pose = get_Pose(new_pose)
            path.poses.append(pose_stamp)
    return path


class Results():

    def __init__(self,
                 exp_idx = 0):

        self.exp_idx = exp_idx
        # ---- Saving path --------------#
        self.csv_saving_path_sslam = '/home/melodic/data/complex_sslam_data/Error_'
        self.csv_saving_path_hector = '/home/melodic/data/complex_hector_data/Error_'
        self.csv_saving_dt_sslam = '/home/melodic/data/complex_sslam_dt/dt_'
        self.csv_saving_dt_hector = '/home/melodic/data/complex_hector_dt/dt_'

        # topics names:
        self.agent_name_topic = '/gazebo/model_states'
        self.hec_slam_topic = 'slam_out_pose'
        self.sslam_topic = '/SSLAM/pose'
        self.exp_idx_topic = '/Exp_index'
        self.path_history_topic = '/SSLAM/path_history'
        self.real_path_history_topic = '/SSLAM/real_path_history'
        self.hector_path_history_topic = '/hector/path_history'


        # pose initialization
        self.real_pose = np.zeros(3)
        self.sslam_pose = np.zeros(3)
        self.hector_pose = np.zeros(3)

        # Path initialization
        self.path_real = Path()
        self.path_sslam = Path()
        self.path_hector = Path()

        self.path_real.header.frame_id = 'world'
        self.path_sslam.header.frame_id  = 'map'
        self.path_hector.header.frame_id = 'map'
        # error initialization
        self.sslam_error = 0
        self.hector_error = 0
        self.error_vector = []
        self.hector_error_vector = []
        self.dt_vector = []
        self.hector_dt_vector = []

        self.hector_counter = rospy.Time.now()
        self.sslam_counter = rospy.Time.now()

        self.sslam_dt = 0
        self.hector_dt = 0

        

        # Publishers:
        self.sslam_path_history_publisher = rospy.Publisher(self.path_history_topic , Path , queue_size = 1)
        self.real_path_history_publisher = rospy.Publisher(self.real_path_history_topic , Path , queue_size = 1) 
        self.hector_path_history_publisher = rospy.Publisher( self.hector_path_history_topic, Path , queue_size = 1)

        # Subscribers:
        rospy.Subscriber(self.exp_idx_topic, Int16, self.Exp_idx_callback, queue_size = 1)
        rospy.Subscriber(self.agent_name_topic ,ModelStates, self.real_pose_callback, queue_size = 1)
        rospy.Subscriber(self.hec_slam_topic ,PoseStamped, self.hector_pose_callback, queue_size = 1)
        rospy.Subscriber(self.sslam_topic , PoseStamped , self.sslam_pose_callback,queue_size = 1) 

        rospy.Timer(rospy.Duration(0.5), self.results_taker)

    # Callbacks:
    def Exp_idx_callback(self,msg):
        
        self.exp_idx = msg.data
    
    def real_pose_callback(self,data):
        
        model_name = 'agent'
        try: id = data.name.index(model_name)
        except: raise Exception('Wrong model name')

        self.real_pose = np.zeros(3)
        self.real_pose[0] = data.pose[id].position.x
        self.real_pose[1] = data.pose[id].position.y

        x = data.pose[id].orientation.x
        y = data.pose[id].orientation.y
        z = data.pose[id].orientation.z
        w = data.pose[id].orientation.w

        r = Rotation.from_quat([x, y, z, w])
        self.real_pose[2] = r.as_euler('zxy')[0]

        
        
    def hector_pose_callback(self,data):

        self.hector_dt = data.header.stamp.to_sec() - self.hector_counter.to_sec()
        self.hector_counter = data.header.stamp
        self.hector_pose = np.zeros(3)
        self.hector_pose[0] = data.pose.position.x
        self.hector_pose[1] = data.pose.position.y

        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w

        r = Rotation.from_quat([x, y, z, w])
        self.hector_pose[2] = r.as_euler('zxy')[0]
        

    def sslam_pose_callback(self,data):

        self.sslam_dt = data.header.stamp.to_sec() - self.sslam_counter.to_sec()
        self.sslam_counter = data.header.stamp
        self.sslam_pose = np.zeros(3)
        self.sslam_pose[0] = data.pose.position.x
        self.sslam_pose[1] = data.pose.position.y

        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z
        w = data.pose.orientation.w

        r = Rotation.from_quat([x, y, z, w])
        self.sslam_pose[2] = r.as_euler('zxy')[0]

    
    def get_model_pose_from_gazebo(self):
        return self.real_pose
    
    def get_model_pose_from_hector(self):
        return self.hector_pose , self.hector_dt
    
    def get_model_pose_from_sslam(self):
        return self.sslam_pose , self.sslam_dt



    def sslam_data_to_csv(self , name):
    
        
        try: 
            df = pandas.read_csv(name)
            dic = {'Error': self.error_vector}
            df = pandas.DataFrame(dic)
            df.to_csv(name ,index = False)
            
        except:
            print('File does not exist, generate the file.')
            dic = {'Error': (self.error_vector)}
            df = pandas.DataFrame(dic)
            df.to_csv(name , index = False)

    
    def hector_data_to_csv(self , name):
    
        
        try: 
            df = pandas.read_csv(name)
            dic = {'Error': self.hector_error_vector}
            df = pandas.DataFrame(dic)
            df.to_csv(name ,index = False)
            
        except:
            print('File does not exist, generate the file.')
            dic = {'Error': (self.hector_error_vector)}
            df = pandas.DataFrame(dic)
            df.to_csv(name , index = False)

    def sslam_dt_to_csv(self , name):
    
        
        try: 
            df = pandas.read_csv(name)
            dic = {'dt': self.dt_vector}
            df = pandas.DataFrame(dic)
            df.to_csv(name ,index = False)
            
        except:
            print('File does not exist, generate the file.')
            dic = {'dt': (self.hector_dt_vector)}
            df = pandas.DataFrame(dic)
            df.to_csv(name , index = False)

    
    def hector_dt_to_csv(self , name):
    
        
        try: 
            df = pandas.read_csv(name)
            dic = {'dt': self.hector_error_vector}
            df = pandas.DataFrame(dic)
            df.to_csv(name ,index = False)
            
        except:
            print('File does not exist, generate the file.')
            dic = {'dt': (self.hector_error_vector)}
            df = pandas.DataFrame(dic)
            df.to_csv(name , index = False)

    def results_taker(self,t):

        real_pose = self.get_model_pose_from_gazebo()
        hector_pose , hector_dt = self.get_model_pose_from_hector()
        sslam_pose , sslam_dt = self.get_model_pose_from_sslam()

        # print(sslam_dt)

        self.path_real = update_path(self.path_real,real_pose , 0,'world')
        self.path_sslam = update_path(self.path_sslam,sslam_pose , 0,'map')
        self.path_hector = update_path(self.path_hector,hector_pose , 0,'map')
        self.publish_path()

        self.sslam_error = np.linalg.norm(np.array([real_pose[0]-sslam_pose[0],real_pose[1]-sslam_pose[1]]))
        self.hector_error = np.linalg.norm(np.array([real_pose[0]-hector_pose[0],real_pose[1]-hector_pose[1]]))

        self.error_vector = np.append(self.error_vector,[self.sslam_error])
        self.hector_error_vector = np.append(self.hector_error_vector,[self.hector_error])

        self.dt_vector = np.append(self.dt_vector,[sslam_dt])
        self.hector_dt_vector = np.append(self.hector_dt_vector,[hector_dt])

        self.sslam_data_to_csv(name =  self.csv_saving_path_sslam +  str(self.exp_idx) + '.csv')
        self.hector_data_to_csv(name =  self.csv_saving_path_hector +  str(self.exp_idx) + '.csv')

        self.sslam_dt_to_csv(name =  self.csv_saving_dt_sslam +  str(self.exp_idx) + '.csv')
        self.hector_dt_to_csv(name =  self.csv_saving_dt_hector +  str(self.exp_idx) + '.csv')

        

    def publish_path(self):
        stamps = rospy.Time.now()
        self.path_real.header.stamp = stamps
        self.path_sslam.header.stamp = stamps
        self.path_hector.header.stamp = stamps
        self.sslam_path_history_publisher.publish(self.path_sslam)
        self.real_path_history_publisher.publish(self.path_real)
        self.hector_path_history_publisher.publish(self.path_hector)


    

if __name__== '__main__':
    rospy.init_node('results', anonymous = True)
    Re = Results()
    rospy.spin()
        