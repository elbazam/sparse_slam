#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros

def get_transform(pose_msg, frame_id, child_frame_id):
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = pose_msg.pose.position.x
    t.transform.translation.y = pose_msg.pose.position.y
    t.transform.translation.z = pose_msg.pose.position.z
    t.transform.rotation.x = pose_msg.pose.orientation.x
    t.transform.rotation.y = pose_msg.pose.orientation.y
    t.transform.rotation.z = pose_msg.pose.orientation.z
    t.transform.rotation.w = pose_msg.pose.orientation.w
    return t


class TfBridge():
    def __init__(self):
        self.tf = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/slam/real_pose", PoseStamped, self.real_pose_callback, queue_size=1)
        rospy.Subscriber("/slam/expected_pose", PoseStamped, self.expected_pose_callback, queue_size=1)

    def real_pose_callback(self, pose_msg):
        t = get_transform(pose_msg, "world", "gt")
        self.tf.sendTransform(t)

    def expected_pose_callback(self, pose_msg):
        t = get_transform(pose_msg, "map", "base_link")
        self.tf.sendTransform(t)
        

rospy.init_node('tf_bridg', anonymous=True)
tra = TfBridge()
rospy.loginfo("tf_bridg node")
rospy.spin()
