#!/usr/bin/env python3

import numpy as np
import cv2
import time
import rospy
from scipy.spatial.transform import Rotation
import copy
from geometry_msgs.msg import Pose2D, PoseStamped, PoseArray, Pose
from object_msgs.msg import Landmark

def rot_mat(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]], dtype = np.float32)
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

def get_PoseStamped(pose, frame_id):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose = get_Pose(pose)
    return pose_msg

def ModelStates2pose2D(models_msg, name):
    idx = np.squeeze(np.argwhere(np.array(models_msg.name) == name))
    r = Rotation.from_quat([models_msg.pose[idx].orientation.x,
                            models_msg.pose[idx].orientation.y,
                            models_msg.pose[idx].orientation.z,
                            models_msg.pose[idx].orientation.w])
    return np.array([models_msg.pose[idx].position.x,models_msg.pose[idx].position.y, r.as_rotvec()[2]])

def ModelStates2poseStamped(models_msg, name, frame_id):
    idx = np.squeeze(np.argwhere(np.array(models_msg.name) == name))
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.position.x = models_msg.pose[idx].position.x
    pose_msg.pose.position.y = models_msg.pose[idx].position.y
    pose_msg.pose.position.z = models_msg.pose[idx].position.z
    pose_msg.pose.orientation.x = models_msg.pose[idx].orientation.x
    pose_msg.pose.orientation.y = models_msg.pose[idx].orientation.y
    pose_msg.pose.orientation.z = models_msg.pose[idx].orientation.z
    pose_msg.pose.orientation.w = models_msg.pose[idx].orientation.w
    return pose_msg

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

def expectation(particles, w):
    if np.sum(w)==0:
        return np.mean(particles, axis = 0)
    else:
        return np.average(particles, axis=0, weights = w)

def transform_objects(objects, pose):
    reg_objects = []
    for obj in objects.objects:
        reg_obj = copy.deepcopy(obj)
        reg_obj.pose.x = obj.pose.z
        reg_obj.pose.y = -obj.pose.x
        reg_obj.pose.z = -obj.pose.y
        R = rot_mat(pose[2])
        z = np.array([reg_obj.pose.x, reg_obj.pose.y])
        xy = R.dot(z.T).T + pose[:2]
        reg_obj.pose.x = xy[0]
        reg_obj.pose.y = xy[1]
        reg_objects.append(reg_obj)
    return reg_objects

def transform_landmarks(landmarks, pose):
    reg_landmarks = []
    for landmark in landmarks:
        reg_landmark = copy.deepcopy(landmark)
        reg_landmark.y = -reg_landmark.y 
        reg_landmark.x = reg_landmark.x + 0.17
        R = rot_mat(pose[2])
        z = np.array([reg_landmark.x, reg_landmark.y])
        xy = R.dot(z.T).T + pose[:2]
        reg_landmark.x = xy[0]
        reg_landmark.y = xy[1]
        reg_landmarks.append(reg_landmark)
    return reg_landmarks

def ExitLines2Landmarks(exit_lines):
    landmarks = []
    for exit_line in exit_lines:
        landmark1 = Landmark()
        landmark2 = Landmark()
        landmark1.x = exit_line.x1
        landmark1.y = exit_line.y1
        landmark2.x = exit_line.x2
        landmark2.y = exit_line.y2
        landmarks.append(landmark1)
        landmarks.append(landmark2)
    return landmarks