#!/usr/bin/env python3

import numpy as np
import cv2
import pandas
from room_slam.api.common import  get_PoseStamped, expectation
import time
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
from object_msgs.msg import Landmark, LandmarkArray, ObjectArray, ExitLine, ExitLineArray
from visualization_msgs.msg import Marker , MarkerArray




import threading


from room_slam.ros.respawn_model import  Respawn_model
from room_slam.api.particle_filter import MCLParticleFilter
from room_slam.api.semantic_map import SemanticMap
from laser_line_extraction.msg import LineSegmentList
from scipy.spatial.transform import Rotation

class SemanticSLAMNode():
    def __init__(self):
        # ---- Init Topics ---- #
        self.objects_markers_topic = rospy.get_param("/SSLAM/objects_markers_topic", '/SSLAM/objects')
        self.corners_markers_topic = rospy.get_param("/SSLAM/corners_markers_topic", '/SSLAM/corners')
        self.exits_markers_topic = rospy.get_param("/SSLAM/exits_markers_topic", '/SSLAM/exits')
        self.walls_markers_topic = rospy.get_param("/SSLAM/walls_markers_topic", '/SSLAM/walls')
        self.rooms_markers_topic = rospy.get_param("/SSLAM/rooms_markers_topic", '/SSLAM/rooms')
        self.graph_markers_topic = rospy.get_param("/SSLAM/graph_markers_topic", '/SSLAM/graph')
        self.pose_topic = rospy.get_param("/SSLAM/pose_topic", '/SSLAM/pose')
        self.particles_topic = rospy.get_param("/SSLAM/particles_topic", '/SSLAM/particles')
        self.path_history_topic = rospy.get_param("/SSLAM/path_history_topic", '/SSLAM/path_history')
        self.real_path_history_topic = rospy.get_param("/SSLAM/real_path_history_topic", '/SSLAM/real_path_history')
        self.object_detector_topic = rospy.get_param("/SSLAM/object_detector_topic", '/object_detector/objects')
        self.corners_detector_topic = rospy.get_param("/SSLAM/corners_detector_topic", '/landmarks/corners' )
        self.exits_detector_topic = rospy.get_param("/SSLAM/exits_detector_topic", '/landmarks/exits')
        self.walls_detector_topic = rospy.get_param("/SSLAM/walls_detector_topic", '/line_segments')
        self.odom_topic = rospy.get_param("/SSLAM/odom_topic", '/odom')
        
        
        
        # ---- Init map ---- #
        self.frame_id = rospy.get_param("/SSLAM/frame_id", 'map')
        self.map = SemanticMap(self.frame_id)
        # ---- Initial first landmarks ----#

        corners_msg = rospy.wait_for_message('/landmarks/corners' , LandmarkArray)
        self._reset_sim()
        while len(corners_msg.landmarks) < 2:
            print("Not enough landmarks to initialize map!")
            corners_msg = rospy.wait_for_message('/landmarks/corners' , LandmarkArray)
            rospy.sleep(1)
        pose = np.array([0,0,0])
        self.map.new_room(pose, corners_msg=corners_msg)

        # ---- Saving path --------------#
        self.csv_saving_path_sslam = '/home/melodic/data/complex_sslam_data/Error_'
        self.csv_saving_path_hector = '/home/melodic/data/complex_hector_data/Error_'
        
        # ---- Init Publishers ---- #
        self.objects_markers_publisher = rospy.Publisher(self.objects_markers_topic , MarkerArray , queue_size = 1)
        self.corners_markers_publisher = rospy.Publisher(self.corners_markers_topic , Marker , queue_size = 1)
        self.exits_markers_publisher = rospy.Publisher(self.exits_markers_topic , Marker , queue_size = 1)
        self.walls_markers_publisher = rospy.Publisher(self.walls_markers_topic , Marker , queue_size = 1)
        self.rooms_markers_publisher = rospy.Publisher(self.rooms_markers_topic , MarkerArray , queue_size = 1)
        self.graph_markers_publisher = rospy.Publisher(self.graph_markers_topic , Marker , queue_size = 1)
        self.pose_publisher = rospy.Publisher(self.pose_topic , PoseStamped , queue_size = 1) 
        self.particles_publisher = rospy.Publisher(self.particles_topic , PoseArray , queue_size = 1) 
        self.sslam_path_history_publisher = rospy.Publisher(self.path_history_topic , Path , queue_size = 1)
        self.real_path_history_publisher = rospy.Publisher(self.real_path_history_topic , Path , queue_size = 1) 
        self.hector_path_history_publisher = rospy.Publisher('/hector/path_history' , Path , queue_size = 1)
        
        
        
        
        # ---- Init MCL ---- #
        self.MCL = MCLParticleFilter(
            Np = rospy.get_param("/SSLAM/MCL/Np", 100),
            Np_min = rospy.get_param("/SSLAM/MCL/Np_min", 100),
            Np_max = rospy.get_param("/SSLAM/MCL/Np_max", 150),
            sigma_v = rospy.get_param("/SSLAM/MCL/sigma_v", 0.7),
            sigma_theta_dot_enc = rospy.get_param("/SSLAM/MCL/sigma_theta_dot_enc", 1.0),
            init_cov = np.diag(rospy.get_param("/SSLAM/MCL/init_cov", [0.001,0.001,0.00005])),
            init_pose = pose,
            N_eff_frac = rospy.get_param("/SSLAM/MCL/N_eff_frac", 0.5),
            regularization_cov =  np.diag(rospy.get_param("/SSLAM/MCL/regularization_cov", [0.003,0.003,0.0015])),
            reset_cov = np.diag(rospy.get_param("/SSLAM/MCL/reset_cov", [0.5,0.5,0.25])))

        # ---- Init Flags ---- #
        self.lock_map_flag = False
        self.user_lock_map_flag = False
        self.new_map_flag = False
        self.global_localization_flag = False
        self.low_evidance_counter = 0
        self.use_saved_map = rospy.get_param("/SSLAM/use_saved_map", False)
        self.saved_map_path = rospy.get_param("/SSLAM/saved_map_path", 'old_map.pkl')

        # --- Initialize global localization (only for saved maps) ---- #
        if self.use_saved_map == True:
            self.map.load_map(self.saved_map_path)
            self.global_localization_flag = True
            self.lock_map_flag = True
            self.user_lock_map_flag = True
        # ---- Init pose ---- #
        self.last_update_pose_corners = pose
        self.last_update_pose_exits = pose
        self.last_update_pose_objects = pose
        self.last_update_pose_ogmap = pose
        self.last_update_pose_walls = pose

        # ---- Public vars ---- #
        self.map_path = rospy.get_param("/SSLAM/map_path", 'new_map.pkl')
        self.path_real = Path()
        self.path_sslam = Path()
        self.path_hector = Path()
        
        self.pose_array = []
        self.path_real.header.frame_id = 'world'
        self.path_sslam.header.frame_id  = self.frame_id
        self.path_hector.header.frame_id = self.frame_id
        
        self.last_time_odom = rospy.Time.now()
        self.thread_lock =  threading.Lock()

        # ---- Error data ----
        self.error = 0
        self.hector_error = 0
        self.error_vector = []
        self.hector_error_vector = []
        self.exp_idx = 0
        self.real_pose = np.zeros(3)
        self.hector_pose = np.zeros(3)
        self.reset = False
        self.agent_name_topic = '/gazebo/model_states'
        self.hec_slam_topic = 'slam_out_pose'
        
        self.laser_scan = LaserScan()

        # ---- Init Subscribers ---- #
        rospy.Timer(rospy.Duration(0.05), self.visualization_callback)
        
        rospy.Subscriber(self.object_detector_topic ,ObjectArray, self.objects_callback, queue_size = 1)
        rospy.Subscriber(self.corners_detector_topic ,LandmarkArray, self.corners_callback, queue_size = 1)
        rospy.Subscriber(self.exits_detector_topic ,ExitLineArray, self.exits_callback, queue_size = 1)
        rospy.Subscriber(self.walls_detector_topic ,LineSegmentList, self.walls_callback, queue_size = 1)
        rospy.Subscriber(self.odom_topic ,Odometry, self.encoders_callback, queue_size = 1)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.user_pose_init, queue_size = 1)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback , queue_size = 1)
        

    def _reset_sim(self):
        '''Gazebo simulation reset command'''
        spawned = Respawn_model()
        while True:
            if spawned:
                break
            else:
                spawned = Respawn_model()

        

    def user_pose_init(self, pose_msg):
        '''Agent pose initialization'''
        pose = np.zeros(3)
        pose[0] = pose_msg.pose.pose.position.x
        pose[1] = pose_msg.pose.pose.position.y
        qu = pose_msg.pose.pose.orientation
        r = Rotation.from_quat([qu.x, qu.y, qu.z, qu.w])
        pose[2] = r.as_euler('zxy')[0]
        self.MCL.reset_particles(pose)
        
    def laser_callback(self, laser_msg):

        self.laser_scan = laser_msg



    def encoders_callback(self, odom_msg):
        self.thread_lock.acquire()

        self.MCL.predict_enoders(odom_msg)
        pose = self.MCL.expectation()
        
        # self.pose_publisher.publish(get_PoseStamped(pose, self.frame_id))  

        # dt = odom_msg.header.stamp.to_sec() - self.last_time_odom.to_sec()
        
        # if dt > 0.001:
            
        self.last_time_odom = odom_msg.header.stamp
        
        self.pose_array.append(pose)
        if len(self.pose_array) > 10:
            self.pose_array.pop(0)
        if self.user_lock_map_flag == True:
            if np.trace(self.MCL.cov()) < 0.1:
                self.user_lock_map_flag = False

        self.thread_lock.release()
        
    def walls_callback(self, lines_msg):      
        self.thread_lock.acquire()
        if len(self.pose_array) > 1:
            pose = self.pose_array[-2]
        else:
            pose = self.MCL.expectation()
            
        delta_pose = np.linalg.norm(self.last_update_pose_walls - pose)
        if delta_pose > 0.005:
            self.last_update_pose_walls = pose
            L = np.zeros(self.MCL.Np)
            for ii in range(self.MCL.Np):
                L[ii] = self.map.likelihood_walls(self.MCL.particles[ii], lines_msg, global_localization=False)
            if np.mean(L) > 0.004:
                self.MCL.update_particles(L)
                pose = self.MCL.expectation()
            self.pose_publisher.publish(get_PoseStamped(pose, self.frame_id)) 
            
            if np.abs(self.MCL.v) < 0.15 and np.abs(self.MCL.theta_dot_enc) < 0.15 and self.lock_map_flag == False and self.user_lock_map_flag == False:
                
                
                self.map.update_room_walls(pose, lines_msg)
            else:
                print("Can't update map ("+str(self.map.current_room_id)+"), slow down.")
        self.thread_lock.release()
        
    def exits_callback(self, exits_msg):
        # self.thread_lock.acquire()
        if len(self.pose_array) > 1:
            pose = self.pose_array[-2]
        else:
            pose = self.MCL.expectation()
        delta_pose = np.linalg.norm(self.last_update_pose_exits - pose)
        if delta_pose > 0.005:
            self.last_update_pose_exits = pose
            L = np.zeros(self.MCL.Np)
            for ii in range(self.MCL.Np):
                L[ii] = self.map.likelihood_room_exits(self.MCL.particles[ii], exits_msg, global_localization=False)
            if np.mean(L) > 0.01:
                self.MCL.update_particles(L)
                pose = self.MCL.expectation()
            self.pose_publisher.publish(get_PoseStamped(pose, self.frame_id))
            
            if np.abs(self.MCL.v) < 0.15 and np.abs(self.MCL.theta_dot_enc) < 0.15 and self.lock_map_flag == False and self.user_lock_map_flag == False:
                
                scan = self.laser_scan
                try:
                    self.map.update_room_exits(pose, exits_msg, scan)
                except rospy.ROSException:
                    self.map.update_room_exits(pose, exits_msg)
                    print("!")
            else:
                print("Can't update map ("+str(self.map.current_room_id)+"), slow down.")
        # self.thread_lock.release()

    def corners_callback(self, landmarks_msg):
        self.thread_lock.acquire()
        if len(self.pose_array) > 1:
            pose = self.pose_array[-2]
        else:
            pose = self.MCL.expectation()
        if self.new_map_flag == True and np.abs(self.MCL.v) < 0.15 and np.abs(self.MCL.theta_dot_enc) < 0.15 and self.user_lock_map_flag == False:
            try:
                self.map.new_room(pose, landmarks_msg)
                print("New room created (id: "+str(self.map.current_room_id)+")")
                self.new_map_flag = False
                self.low_evidance_counter = 0
            except:
                print('Could not update the last room location, will try again soon')
        elif self.new_map_flag == True:
            print("Can't create a new map, slow down.")
        else:
            delta_pose = np.linalg.norm(self.last_update_pose_corners - pose)
            if delta_pose > 0.005: 
                self.last_update_pose_corners = pose
                self.map.update_room_pdf()
                L = np.zeros(self.MCL.Np)
                for ii in range(self.MCL.Np):
                    L[ii] = self.map.likelihood_room_corners(self.MCL.particles[ii], landmarks_msg, global_localization=False)
                if np.mean(L) > 0.01:
                    self.MCL.update_particles(L)
                    pose = self.MCL.expectation()
                    self.low_evidance_counter = 0
                else:
                    print("Warning, low evidance!")
                    self.low_evidance_counter += 1
                    if self.low_evidance_counter > 100:
                        print("Very low evidance, please roll back or create a new map! Locking current map.")
                        self.lock_map_flag = True
                self.pose_publisher.publish(get_PoseStamped(pose, self.frame_id))
                if np.abs(self.MCL.v) < 0.15 and np.abs(self.MCL.theta_dot_enc) < 0.15 and self.lock_map_flag == False and self.user_lock_map_flag == False:

                    
                    self.map.update_room_corners(pose, landmarks_msg)
                else:
                    print("Can't update map ("+str(self.map.current_room_id)+"), slow down.")
        self.thread_lock.release()   

    def objects_callback(self, objects_msg):
        self.thread_lock.acquire()
        pose = self.MCL.expectation()
        delta_pose = np.linalg.norm(self.last_update_pose_objects - pose)
        if delta_pose > 0.005:
            
            self.last_update_pose_objects = pose
            L = np.zeros(self.MCL.Np)
            for ii in range(self.MCL.Np):
                L[ii] = self.map.likelihood_room_objects(self.MCL.particles[ii], objects_msg , global_localization=False)
            if np.mean(L) > 0.01:
                self.MCL.update_particles(L)
                pose = self.MCL.expectation()
            self.pose_publisher.publish(get_PoseStamped(pose, self.frame_id))
            if np.abs(self.MCL.v) < 0.15 and np.abs(self.MCL.theta_dot_enc) < 0.15 and self.lock_map_flag == False and self.user_lock_map_flag == False:
                
                self.map.update_room_objects(pose, objects_msg)
            else:
                print("Can't update map ("+str(self.map.current_room_id)+"), slow down.")
        if self.global_localization_flag:
            obj = objects_msg.objects[0]
            objects_pose = np.array([obj.pose.z, -obj.pose.x, -obj.pose.y])
            object_cls = obj.cls
            d = np.linalg.norm(objects_pose[:2])
            theta = np.arctan2(objects_pose[1], objects_pose[0]) - np.pi/2
            relevent_objects = []
            for ii in range(self.map.n_rooms): 
                if len(self.map.rooms[ii].objects.poses) == 0:
                    continue
                elif self.map.rooms[ii].objects.poses.shape == (3,):
                    self.map.rooms[ii].objects.poses = np.array([self.map.rooms[ii].objects.poses])                   
                for jj in range(len(self.map.rooms[ii].objects.poses)):
                    print(self.map.rooms[ii].objects.C[jj])
                    if self.map.rooms[ii].objects.C[jj] == object_cls:
                        relevent_objects.append(self.map.rooms[ii].objects.poses[jj])
            print(relevent_objects) 
            if len(relevent_objects) > 0:
                self.MCL.global_localization(np.array(relevent_objects), d, theta)
                self.global_localization_flag = False


        self.thread_lock.release()
    

    def visualization_callback(self, t):  
        
        pose = self.MCL.expectation()
        
        # self.particles_publisher.publish(self.MCL.visualize_particles(self.frame_id)) 
        if self.map.is_walls_available():
            self.walls_markers_publisher.publish(self.map.visualize_walls())
        if self.map.is_objects_available():
            self.objects_markers_publisher.publish(self.map.visualize_objects())
        if self.map.is_corners_available():
            self.corners_markers_publisher.publish(self.map.visualize_corners())
        if self.map.is_exits_available():
            self.exits_markers_publisher.publish(self.map.visualize_exits())
        if self.map.is_rooms_available():
            #self.rooms_ogmap_publisher.publish(self.map.visualize_ogmap())
            for i in range(3):
                try:
                    self.rooms_markers_publisher.publish(self.map.visualize_rooms(visualize_all_rooms = True))
                    self.graph_markers_publisher.publish(self.map.visualize_graph())
                except:
                    print("could not visualize room")
            # --- Chack for new rooms availibility ---- #
            
            new_room_advice, move_to_room_advice, update_map_advice = self.map.advise_room(pose)
            self.lock_map_flag = (not update_map_advice)
            if new_room_advice:
                print("Advising new room!")
                self.new_map_flag = True
            elif self.map.current_room_id != move_to_room_advice:
                print("Advising change room from "+str(self.map.current_room_id)+" to "+str(move_to_room_advice))
                self.map.new_edge(self.map.current_room_id, move_to_room_advice, pose)
                self.map.current_room_id = move_to_room_advice
                



if __name__== '__main__':
    rospy.init_node('sslam_node', anonymous = True)
    slam = SemanticSLAMNode()
    
    rospy.spin()

