from visualization_msgs.msg import Marker, MarkerArray
import rospy
import numpy as np
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation

def corners_marker(id, poses, ns, color, frame_id):
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


def text_marker(name, id, pose, frame_id, color):
    marker_name = Marker()
    marker_name.header.frame_id = frame_id
    marker_name.header.stamp = rospy.Time.now()
    # Orientation of the text
    marker_name.pose.orientation.x = 0
    marker_name.pose.orientation.y = 0
    marker_name.pose.orientation.z = 0
    marker_name.pose.orientation.w = 1
    # Colore of text
    marker_name.color.r = color[0]
    marker_name.color.g = color[1]
    marker_name.color.b = color[2]
    marker_name.color.a = 2.0
    # Rest of things:
    marker_name.id = id
    marker_name.type = 9
    marker_name.action = 0
    marker_name.lifetime.secs = 1
    marker_name.pose.position.x = pose[0]
    marker_name.pose.position.y = pose[1]
    marker_name.pose.position.z = pose[2]
    # Size of the text
    marker_name.scale.x = 0.5
    marker_name.scale.y = 0.5
    marker_name.scale.z = 0.5
    marker_name.text = name
    marker_name.ns = name + str(id)
    return marker_name


def objects_marker(objects, frame_id, color):
    list_marker = MarkerArray()
    list_marker.markers = []
    if objects.poses.shape == (3,):
        marker_temp = text_marker(objects.C[0], 0, objects.poses, frame_id, color)
        list_marker.markers.append(marker_temp)
    else:
        for ii in range(len(objects.poses)):
            point = np.array(objects.poses[ii])
            marker_temp = text_marker(objects.C[ii], ii, point, frame_id, color)
            list_marker.markers.append(marker_temp)
    return list_marker

def exits_marker(id, exits, ns, color, frame_id):
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
    for ii in range(0, exits.shape[0]-1,2):
        point_temp1 = Point()
        point_temp1.x = exits[ii,0]
        point_temp1.y = exits[ii,1]
        point_temp1.z = 0
        marker_temp.points.append(point_temp1)
        point_temp2 = Point()
        point_temp2.x = exits[ii+1,0]
        point_temp2.y = exits[ii+1,1]
        point_temp2.z = 0
        marker_temp.points.append(point_temp2)
    # Size of the text
    marker_temp.scale.x = 0.1
    marker_temp.scale.y = 0.1
    marker_temp.scale.z = 0.1
    marker_temp.ns = ns
    return marker_temp

def room_marker(room, frame_id, color, ns, id):
    mux, muy, theta,a,b = room
    r = Rotation.from_euler('z',theta, degrees=False)
    marker_name = Marker()
    marker_name.header.frame_id = frame_id
    marker_name.header.stamp = rospy.Time.now()
    # Orientation of the room
    marker_name.pose.orientation.x =r.as_quat()[0]
    marker_name.pose.orientation.y = r.as_quat()[1]
    marker_name.pose.orientation.z = r.as_quat()[2]
    marker_name.pose.orientation.w = r.as_quat()[3]
    # Colore of text
    marker_name.color.r = color[0]
    marker_name.color.g = color[1]
    marker_name.color.b = color[2]
    marker_name.color.a = 0.5       
    # Rest of things:
    marker_name.id = id
    marker_name.type = 1
    marker_name.action = 0
    marker_name.lifetime.secs = 10
    marker_name.pose.position.x = mux
    marker_name.pose.position.y = muy
    marker_name.pose.position.z = 0
    # Size of the text
    marker_name.scale.x = a
    marker_name.scale.y = b
    marker_name.scale.z = 0.01
    marker_name.ns = ns + str(id)
    return marker_name

    