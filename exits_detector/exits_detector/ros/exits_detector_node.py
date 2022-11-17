#!/usr/bin/env python3

import numpy as np
import cv2
import time
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegmentList
from exits_detector.api import find_optional_exits, filter_exits, visualize, find_intersections
from geometry_msgs.msg import Pose, PoseArray, Point
from object_msgs.msg import Landmark, LandmarkArray, ExitLine, ExitLineArray
from visualization_msgs.msg import Marker , MarkerArray
import rospy

def new_marker(id, poses, ns, color, frame_id):
    marker_temp = Marker()
    marker_temp.header.frame_id = frame_id 
    marker_temp.header.stamp = rospy.Time.now()
    # Orientation of the text
    marker_temp.pose.orientation.x = 0
    marker_temp.pose.orientation.y = 0
    marker_temp.pose.orientation.z = 0
    marker_temp.pose.orientation.w = 1
    # Colore of text
    marker_temp.color.r = color[0]
    marker_temp.color.g = color[1]
    marker_temp.color.b = color[2]
    marker_temp.color.a = 2.0
    # Rest of things:
    marker_temp.id = id
    marker_temp.type = 8
    marker_temp.action = 0
    marker_temp.lifetime.secs = 1
    for ii in range(len(poses)):
        point_temp = Point()
        point_temp.x = poses[ii,0]
        point_temp.y = poses[ii,1]
        point_temp.z = 0
        marker_temp.points.append(point_temp)
    # Size of the text
    marker_temp.scale.x = 0.1
    marker_temp.scale.y = 0.1
    marker_temp.scale.z = 0.1
    marker_temp.ns = ns
    return marker_temp

def new_line_marker(id, lines, ns, color, frame_id):
    marker_temp = Marker()
    marker_temp.header.frame_id = frame_id 
    marker_temp.header.stamp = rospy.Time.now()
    # Orientation of the text
    marker_temp.pose.orientation.x = 0
    marker_temp.pose.orientation.y = 0
    marker_temp.pose.orientation.z = 0
    marker_temp.pose.orientation.w = 1
    # Colore of text
    marker_temp.color.r = color[0]
    marker_temp.color.g = color[1]
    marker_temp.color.b = color[2]
    marker_temp.color.a = 2.0
    # Rest of things:
    marker_temp.id = id
    marker_temp.type = 5
    marker_temp.action = 0
    marker_temp.lifetime.secs = 1
    for ii in range(len(lines)):
        point_temp1 = Point()
        point_temp1.x = lines[ii,0,0]
        point_temp1.y = lines[ii,0,1]
        point_temp1.z = 0
        marker_temp.points.append(point_temp1)
        point_temp2 = Point()
        point_temp2.x = lines[ii,1,0]
        point_temp2.y = lines[ii,1,1]
        point_temp2.z = 0
        marker_temp.points.append(point_temp2)
    # Size of the text
    marker_temp.scale.x = 0.1
    marker_temp.scale.y = 0.1
    marker_temp.scale.z = 0.1
    marker_temp.ns = ns
    return marker_temp


class exits_detector_node():
    def __init__(self,
                 corners_landmarks_topic = '/landmarks/corners',
                 exits_landmarks_topic = '/landmarks/exits',
                 lines_topic = '/line_segments',
                 scan_topic = '/scan',
                 exits_marker_topic = '/exits_marker',
                 exits_lines_marker_topic = '/exits_lines_marker',
                 corners_marker_topic = '/corners_marker',
                 frame_id='base_laser',
                 publish_markers = True,
                 max_exit_size = 2.2,
                 th = 0.8,
                 res = (0.05, 0.05),
                 metric_size = (8, 8)):
        self.publish_markers = publish_markers
        self.max_exit_size = max_exit_size
        self.th = th
        self.res = res
        self.metric_size = metric_size
        self.corners_landmarks_pub = rospy.Publisher(corners_landmarks_topic,LandmarkArray, queue_size = 1)
        self.exits_landmarks_pub = rospy.Publisher(exits_landmarks_topic,ExitLineArray, queue_size = 1)
        if self.publish_markers:
            self.exits_marker_pub = rospy.Publisher(exits_marker_topic,Marker, queue_size = 1)
            self.exits_lines_marker_pub = rospy.Publisher(exits_lines_marker_topic,Marker, queue_size = 1)
            self.corners_marker = rospy.Publisher(corners_marker_topic,Marker, queue_size = 1)
        self.frame_id = frame_id
        self.scan_topic = scan_topic
        self.scan = rospy.wait_for_message(self.scan_topic,LaserScan)
        rospy.Subscriber(lines_topic,LineSegmentList, self.line_segments_callback, queue_size = 1)
        rospy.Subscriber(scan_topic,LaserScan, self.scan_callback, queue_size = 1)

    def filter_corners(self, corners, range_max = 7):
        
        
        filtered_corners = []
        for ii in range(len(corners)):
            if np.linalg.norm(corners[ii,:]) < range_max:
                filtered_corners.append(corners[ii,:])
        # idx = np.where(np.linalg.norm(corners,axis=1)<range_max)
        # filtered_corners = corners[idx,:]
        return np.array(filtered_corners)

    def scan_callback(self,msg):
        self.scan = msg

    def line_segments_callback(self, lines_msg):
        
        # ---- Find and handle corners ---- #
        scan = self.scan
        corners = find_intersections(lines_msg, self.th)
        corners = self.filter_corners(corners)
        if len(corners) > 0:
            corners_msg = LandmarkArray()
            corners_msg.header.stamp = rospy.Time.now()
            corners_msg.header.frame_id = self.frame_id
            for ii in range(len(corners)):
                landmark = Landmark()
                landmark.x = corners[ii,0]
                landmark.y = corners[ii,1]
                corners_msg.landmarks.append(landmark)
            self.corners_landmarks_pub.publish(corners_msg)
            if self.publish_markers:
                markers_corners = new_marker(id = 0, poses = corners, ns = 'corners', color = [0.5,1,0.5], frame_id = self.frame_id)
                self.corners_marker.publish(markers_corners)

        # ---- Find and handle exits ---- #
        line_intersections, exit_lines = find_optional_exits(lines_msg, self.th, self.max_exit_size)
        exits_idxs = filter_exits(line_intersections, exit_lines,scan, self.res, self.metric_size)
        if len(exits_idxs) > 0:
            exits_msg = ExitLineArray()
            exits_msg.header.stamp = rospy.Time.now()
            exits_msg.header.frame_id = self.frame_id

            exits = line_intersections[exits_idxs]
            exits_lines = exit_lines[exits_idxs]
            for ii in range(len(exits)):
                exit_temp = ExitLine()
                exit_temp.x1 = exits_lines[ii,0,0]
                exit_temp.y1 = exits_lines[ii,0,1]
                exit_temp.x2 = exits_lines[ii,1,0]
                exit_temp.y2 = exits_lines[ii,1,1]
                exits_msg.exit_lines.append(exit_temp)
            self.exits_landmarks_pub.publish(exits_msg)

            if self.publish_markers:
                markers_exits = new_marker(id = 0, poses = exits, ns = 'exits', color = [1,0,0], frame_id = self.frame_id)
                marker_exits_lines = new_line_marker(id = 0, lines = exits_lines, ns = 'exits_lines', color = [1,1,1], frame_id = self.frame_id)
                self.exits_lines_marker_pub.publish(marker_exits_lines)
                self.exits_marker_pub.publish(markers_exits)
            
        
        
    
   

def shutdown():
    cv2.destroyAllWindows() 

if __name__ == "__main__":
    rospy.init_node('exit_detector_node', anonymous=True)
    rospy.on_shutdown(shutdown)
    ED = exits_detector_node()
    rospy.spin()
    rospy.signal_shutdown("exit_detector_node shutdown...")