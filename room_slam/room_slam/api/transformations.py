import numpy as np


def rot_mat(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]], dtype = np.float32)


def landmarks_msg2array(landmarks_msg):
    z = []
    for landmark in landmarks_msg.landmarks: 
        z.append([landmark.x + 0.17, -landmark.y])
    return np.array(z, dtype = np.float)

def exits_msg2array(exits_msg):
    z = []
    for exit_line in exits_msg.exit_lines: 
        z.append([exit_line.x1 + 0.17, -exit_line.y1])
        z.append([exit_line.x2 + 0.17, -exit_line.y2])
    return np.array(z, dtype = np.float)

def lines_msg2array(lines_msg):
    z_start = []
    z_end = []
    for ii in range(len(lines_msg.line_segments)):
        z_start.append([lines_msg.line_segments[ii].start[0] + 0.17, -lines_msg.line_segments[ii].start[1]])
        z_end.append([lines_msg.line_segments[ii].end[0] + 0.17, -lines_msg.line_segments[ii].end[1]])
    return np.array(z_start, dtype = np.float), np.array(z_end, dtype = np.float)

def objects_msg2array(objects_msg):
    z = []
    for obj in objects_msg.objects: 
        z.append([obj.pose.z, -obj.pose.x, -obj.pose.y])
    return np.array(z, dtype = np.float)

def transform2d(z, pose):
    R = rot_mat(pose[2])
    xy = R.dot(z.T).T + pose[:2]
    return xy

def transform3d(z, pose):
    R = rot_mat(pose[2])
    xy = R.dot(z[:,:2].T).T + pose[:2]
    if xy.shape[0] == 1:
        xyz = np.array([xy[0,0], xy[0,1], z[0,2]])
    else:
        xyz = np.concatenate((xy, np.array([z[:,2]]).T), axis = 1)
    return xyz

def transform_scan_msg2array(scan, pose):
    theta = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    pcl = np.empty((len(scan.ranges), 2))
    ranges = np.array(scan.ranges)
    ranges[np.isinf(ranges.copy())==True] = scan.range_max
    pcl[:,0] = np.multiply(ranges, np.cos(theta))
    pcl[:,1] = np.multiply(ranges, np.sin(theta))
    pcl[:,0] = pcl[:,0] + 0.17
    pcl[:,1] = -pcl[:,1]
    return transform2d(pcl, pose)