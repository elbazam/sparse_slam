#!/usr/bin/env python3

import numpy as np
import cv2
import time
import rospy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import  Pose, PoseArray


def particles2PoseArray(particles, frame_id):
    particles_msg = PoseArray()
    particles_msg.header.frame_id = frame_id
    particles_msg.header.stamp = rospy.Time.now()
    for ii in range(len(particles)):
        particle_temp = Pose()
        particle_temp.position.x = particles[ii,0]
        particle_temp.position.y = particles[ii,1]
        particle_temp.position.z = 0
        r = Rotation.from_euler('z',particles[ii,2], degrees=False)
        particle_temp.orientation.x = r.as_quat()[0]
        particle_temp.orientation.y = r.as_quat()[1]
        particle_temp.orientation.z = r.as_quat()[2]
        particle_temp.orientation.w = r.as_quat()[3]
        particles_msg.poses.append(particle_temp)
    return particles_msg




class MCLParticleFilter():
    def __init__(self,
     Np,
     Np_min,
     Np_max,
     sigma_v,
     sigma_theta_dot_enc,
     init_cov, 
     init_pose, 
     N_eff_frac, 
     regularization_cov,
     reset_cov):
        self.last_time_enc = rospy.Time.now()
        self.last_time_IMU = rospy.Time.now()
        self.v = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_posex = 0.0
        self.last_posey = 0.0
        self.theta_dot_enc = 0.0
        self.theta_dot_IMU = 0.0
        self.sigma_v = sigma_v
        self.Np = Np
        self.Np_min = Np_min
        self.Np_max = Np_max
        self.sigma_theta_dot_enc = sigma_theta_dot_enc
        self.w = np.ones(Np)/Np
        self.N_eff_frac = N_eff_frac
        self.regularization_cov = regularization_cov
        self.particles = np.random.multivariate_normal(init_pose, init_cov, self.Np)
        self.reset_cov = reset_cov


    def predict_enoders(self, odom_msg):
        
        dt = odom_msg.header.stamp.to_sec() - self.last_time_enc.to_sec() 
        self.vx = odom_msg.pose.pose.position.x - self.last_posex
        self.vy = odom_msg.pose.pose.position.y - self.last_posey
        eps_v = np.random.normal(1.0, dt*self.sigma_v, self.Np)
        eps_theta_dot = np.random.normal(1.0, dt*self.sigma_theta_dot_enc, self.Np)
        
        self.particles[:,0] = self.particles[:,0] + self.vx*eps_v
        self.particles[:,1] = self.particles[:,1] + self.vy*eps_v
        self.particles[:,2] = self.particles[:,2] + dt*self.theta_dot_enc*eps_theta_dot
    
    
        self.last_posex = odom_msg.pose.pose.position.x
        self.last_posey = odom_msg.pose.pose.position.y
        self.theta_dot_enc = odom_msg.twist.twist.angular.z
        self.v = odom_msg.twist.twist.linear.x
        self.last_time_enc = odom_msg.header.stamp
    
    def global_localization(self, objects_poses, d, theta): # objects_poses: n X 2, d: 1
        self.Np = self.Np_max
        self.particles = np.empty((self.Np, 3))
        for ii in range(self.Np_max):
            obj_idx = np.random.choice(len(objects_poses))
            alpha = np.random.uniform(0, 2*np.pi)
            self.particles[ii,2] = 3*np.pi/2 + alpha + theta + np.random.normal(0,0.1)
            self.particles[ii,0] = objects_poses[obj_idx,0] + d*np.cos(alpha) + np.random.normal(0,0.3)
            self.particles[ii,1] = objects_poses[obj_idx,1] + d*np.sin(alpha) + np.random.normal(0,0.3)
        self.w = np.ones(self.Np)/self.Np
    

    def dynamicNp(self):
        tr = np.trace(np.cov(self.particles.T, aweights=self.w))
        new_Np = int(self.Np_min + tr*(self.Np_max - self.Np_min)/10)
        if new_Np>self.Np_max: new_Np = self.Np_max
        if new_Np<self.Np_min: new_Np = self.Np_min
        idxs = np.random.choice(self.Np, new_Np, p = self.w)
        eps_state = np.random.multivariate_normal(np.zeros(3), self.regularization_cov, new_Np)
        self.particles = self.particles[idxs] + eps_state
        self.w = np.ones(new_Np)/new_Np
        self.Np = new_Np

    def resample(self):
        N_eff = 1/np.sum(self.w**2)
        if N_eff < self.Np * self.N_eff_frac:
            idxs = np.random.choice(self.Np, self.Np, p = self.w)
            eps_state = np.random.multivariate_normal(np.zeros(3), self.regularization_cov, self.Np)
            self.particles = self.particles[idxs] + eps_state
            self.w = np.ones(self.Np)/self.Np
            self.dynamicNp()

    def update_particles(self, L):
        if np.sum(L) == 0 or  len(L) != len(self.particles):
             print("Likelihood problem")
        else:
            self.w = self.w * L
            if np.sum(self.w) == 0:
                print("Likelihood problem, reseting weights...")
                self.w = np.ones(self.Np)/self.Np
            else:    
                self.w = self.w/np.sum(self.w)
                self.resample()

    def expectation(self):
        if np.sum(self.w)==0:
            return np.mean(self.particles, axis = 0)
        else:
            return np.average(self.particles, axis=0, weights = self.w)
    
    def cov(self):
        if np.sum(self.w)==0:
            return np.cov(self.particles.T)
        else:
            return np.cov(self.particles.T, aweights=self.w)
    
    def reset_particles(self, pose = None):
        if pose is None:
            pose = self.expectation()
        self.particles = np.random.multivariate_normal(pose, self.reset_cov, self.Np)
        self.w = np.ones(self.Np)/self.Np

    def visualize_particles(self, frame_id):
        return particles2PoseArray(self.particles, frame_id)


        

