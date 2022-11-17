#!/usr/bin/env python3
import numpy as np
import rospy
from skimage.draw import line 

global problematic_angles
problematic_angles = rospy.get_param('/Prob_angles/prob_angles')

def find_optional_exits(lines_msg, th, max_exit_size = 3, min_exit_size = 1):
    global problematic_angles
    line_intersections = []
    exit_lines = []
    
    for ii in range(len(lines_msg.line_segments)):
        line1 = (lines_msg.line_segments[ii].start,lines_msg.line_segments[ii].end)
        for jj in range(ii+1, len(lines_msg.line_segments)):
            if ii != jj:
                line2 = (lines_msg.line_segments[jj].start,lines_msg.line_segments[jj].end)
                intersection, phi , _ = line_intersection(line1,line2)
                
                
                # angle = np.arctan2(intersection[1],intersection[0])
                # if angle < problematic_angles[0] or angle > problematic_angles[1]:
                #     continue
                
                
                if _is_intesection_inside(intersection,np.array(line1[0]), np.array(line1[1]), th) == True and _is_intesection_inside(intersection,np.array(line2[0]), np.array(line2[1]), th) == False:
                    pts = np.array([line2[0], line2[1]])
                    d = np.linalg.norm(pts - intersection, axis = 1)
                    idx = np.argmin(d)
                    if np.linalg.norm(intersection - pts[idx]) < max_exit_size and np.linalg.norm(intersection - pts[idx]) > min_exit_size:
                        line_intersections.append((intersection + pts[idx])/2)
                        exit_lines.append([intersection, pts[idx]])
                elif _is_intesection_inside(intersection,np.array(line1[0]), np.array(line1[1]), th) == False and _is_intesection_inside(intersection,np.array(line2[0]), np.array(line2[1]), th) == True:
                    pts = np.array([line1[0], line1[1]])
                    d = np.linalg.norm(pts - intersection, axis = 1)
                    idx = np.argmin(d)
                    if np.linalg.norm(intersection - pts[idx]) < max_exit_size and np.linalg.norm(intersection - pts[idx]) > min_exit_size:
                        line_intersections.append((intersection + pts[idx])/2)
                        exit_lines.append([intersection, pts[idx]])
                elif _is_intesection_inside(intersection,np.array(line1[0]), np.array(line1[1]), th) == False and _is_intesection_inside(intersection,np.array(line2[0]), np.array(line2[1]), th) == False:
                    pts1 = np.array([line1[0], line1[1]])
                    pts2 = np.array([line2[0], line2[1]])
                    d1 = np.linalg.norm(pts1 - intersection, axis = 1)
                    d2 = np.linalg.norm(pts2 - intersection, axis = 1)
                    idx1 = np.argmin(d1)
                    idx2 = np.argmin(d2)
                    if np.linalg.norm(pts2[idx2] - pts1[idx1]) < max_exit_size and np.linalg.norm(pts2[idx2] - pts1[idx1]) > min_exit_size:
                        line_intersections.append((pts1[idx1] + pts2[idx2])/2)
                        exit_lines.append([pts1[idx1], pts2[idx2]])
                elif _is_lines_con(line1, line2, 0.1):
                    pts1 = np.array([line1[0], line1[1]])
                    pts2 = np.array([line2[0], line2[1]])
                    d1 = np.linalg.norm(pts1 - intersection, axis = 1)
                    d2 = np.linalg.norm(pts2 - intersection, axis = 1)
                    idx1 = np.argmin(d1)
                    idx2 = np.argmin(d2)
                    if np.linalg.norm(pts2[idx2] - pts1[idx1]) < max_exit_size and np.linalg.norm(pts2[idx2] - pts1[idx1]) > min_exit_size:
                        line_intersections.append((pts1[idx1] + pts2[idx2])/2)
                        exit_lines.append([pts1[idx1], pts2[idx2]])

                
    line_intersections = np.array(line_intersections)
    exit_lines = np.array(exit_lines)
    return line_intersections, exit_lines




def find_intersections(lines_msg, th):
    intersections = []
    for ii in range(len(lines_msg.line_segments) ):
        line1 = (lines_msg.line_segments[ii].start,lines_msg.line_segments[ii].end)
        for jj in range(ii+1,len(lines_msg.line_segments)):
            if ii != jj:
                line2 = (lines_msg.line_segments[jj].start,lines_msg.line_segments[jj].end)
                intersection, theta , col_is_possible = line_intersection(line1,line2)
                if np.absolute(theta) < 60 or not col_is_possible or (_is_intesection_inside(intersection,np.array(line1[0]), np.array(line1[1]), th) == False and _is_intesection_inside(intersection,np.array(line2[0]), np.array(line2[1]), th) == False):
                    continue
                else:
                    intersections.append(intersection)
    return np.array(intersections)

def filter_exits(line_intersections , exit_lines, scan, res = (0.05, 0.05), metric_size = (8, 8)):
    theta = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    pcl = np.empty((len(scan.ranges), 2))
    ranges = np.array(scan.ranges)
    ranges[np.isinf(ranges.copy())==True] = scan.range_max
    pcl[:,0] = np.multiply(ranges, np.cos(theta))
    pcl[:,1] = np.multiply(ranges, np.sin(theta))
    shape = np.round(np.array(metric_size)/res).astype(np.int)
    M = np.zeros(shape = shape)
    pcl_dig = np.zeros((len(pcl),2), dtype = np.int)
    
    pcl_dig[:,0] = (np.round(pcl[:,0]/res[0]) + shape[0]//2).astype(np.int)
    pcl_dig[:,1] = (np.round(pcl[:,1]/res[1]) + shape[1]//2).astype(np.int)
    
    pcl_dig = pcl_dig[(pcl_dig[:,0] < shape[0]) * (pcl_dig[:,0] >= 0) * (pcl_dig[:,1] < shape[1]) * (pcl_dig[:,1] >= 0)]
    filtered_line_intersections = []
    if len(pcl_dig)>0:
        M[pcl_dig[:,0], pcl_dig[:,1]] = 1
    for ii in range(len(line_intersections)):
        if _is_there_obstacle(exit_lines[ii,:,:],ranges,scan.angle_increment,scan.angle_min):
            continue

        line_intersections_dig = np.zeros(2, dtype = np.int)
        line_intersections_dig[0] = (np.round(line_intersections[ii,0]/res[0]) + shape[0]//2).astype(np.int)
        line_intersections_dig[1] = (np.round(line_intersections[ii,1]/res[1]) + shape[1]//2).astype(np.int)
        if (line_intersections_dig[0] < shape[0]) and (line_intersections_dig[0] >= 0) and (line_intersections_dig[1] < shape[1]) and (line_intersections_dig[1] >= 0):
            rr, cc = line(shape[0]//2, shape[1]//2, line_intersections_dig[0], line_intersections_dig[1])
            if np.sum(M[rr, cc]) == 0:
                filtered_line_intersections.append(ii)
    return np.array(filtered_line_intersections, dtype = np.int)


def _is_there_obstacle(exit,ranges,angle_increment,min_angle,N = 3):
    '''Sample 3 points on the exit line and 3 corresponding points in the direction of those points
    to find if there is enough space after them so there is no inteference to the exit 
    False - exit
    True - not an exit'''
    x1 = exit[0,0]
    x2 = exit[1,0]
    y1 = exit[0,1]
    y2 = exit[1,1]
    
    if np.abs(x1-x2) < 0.001:
        x_r = np.full(N,x1)
        y_r = np.random.uniform(low = y1,high=y2,size = N)
    else:
        m = (y2-y1)/(x2-x1)
        n = np.full(N,y1 - m*x1)
        x_r = np.random.uniform(low = x1,high=x2,size = N)
        y_r = m*x_r + n
    R_R = np.sqrt(np.power(x_r,2) + np.power(y_r,2))
    Th_R = np.arctan2(y_r,x_r)
    idx_R = np.round((Th_R - min_angle)/angle_increment)
    idx_R = np.array(idx_R,dtype=int)
    test = ranges[idx_R]
    if np.average(test - R_R) < 1: #or np.average(R_R)<0.1:
        return True
    return False




def line_intersection(line1, line2):
    ''' Calculate the intersection between 2 lines, calculate the angle between the lines and check if 2 lines create a corner
    return - [2dpoint , angle , corner creation condition ]'''
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])
    
    ab1 = np.array([xdiff[0] , ydiff[0]])
    ab2 = np.array([xdiff[1] , ydiff[1]])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(ab1, ab2)
    denom = np.linalg.norm(ab1) * np.linalg.norm(ab2)
    if denom == 0:
        raise Exception('One of the lines are too short and is seen as a point')
    
    if np.abs(div / denom) < 1:
        theta = (180/np.pi) * np.arcsin(div / denom)
    else:
        pp = np.sign(div / denom)
        theta = 90 * pp
    if theta == 0 :
       raise Exception('Lines are parallel and will not intersect')
       

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    col = np.array([x, y])
    
    col_possibility = _is_collision_possible(col , line1 , line2)

    return np.array([x, y]), theta , col_possibility

def _is_collision_possible(col,seg1,seg2,threshold = 0.4,dist = 10):
    ''' Return if the collision is close to both segments.'''

    # Make sure x_odd is the min and x_pair is max:
    if seg1[0][0] < seg1[1][0]:
        x1 , x2 = seg1[0][0] , seg1[1][0]
        y1 , y2 = seg1[0][1] , seg1[1][1]
    else:
        x2 , x1 = seg1[0][0] , seg1[1][0]
        y2 , y1 = seg1[0][1] , seg1[1][1]
    
    if seg2[0][0] < seg2[1][0]:
        x3 , x4 = seg2[0][0] , seg2[1][0]
        y3 , y4 = seg2[0][1] , seg2[1][1]
    else:
        x4 , x3 = seg2[0][0] , seg2[1][0]
        y4 , y3 = seg2[0][1] , seg2[1][1]    

    phi12 = np.arctan2(y2 - y1 , x2 - x1)
    phi34 = np.arctan2(y4 - y3 , x4 - x3)

    thresh12 = np.absolute(np.cos(phi12))*threshold 
    thresh34 = np.absolute(np.cos(phi34))*threshold
    
    bound12 = np.array([x1 - thresh12 , x2 + thresh12])
    bound34 = np.array([x3 - thresh34 , x4 + thresh34])

    xc = col[0]

    if _is_in_range(xc , bound12) and _is_in_range(xc,bound34):
        return True 
    return False

def _is_in_range(x,boundary):

    x_min , x_max = boundary[0] , boundary[1]

    if x >= x_min and x <= x_max:
        return True
    return False

def _is_intesection_inside(intesection, A , B, th):
    da = np.linalg.norm(intesection - A)
    db = np.linalg.norm(intesection - B)
    ab = np.linalg.norm(B - A)
    if ab > da - th and ab > db - th:
        return True
    else:
        return False

def _is_lines_con(line1, line2, th):
    try:
        m = (line1[1][1] - line1[0][1]) / (line1[1][0] - line1[0][0])
        
        b = line1[0][1] - m*line1[0][0]
        res1 = np.abs(m*line2[0][0] + b - line2[0][1])
        res2 = np.abs(m*line2[1][0] + b - line2[1][1])
        if res1+res2 < th:
            return True
        else:
            return False
    except:
        # Line has m->infinity
        res1 = np.abs(line2[1][0] - line2[0][0])
        res2 = 0
        if res1+res2 < th:
            return True
        else:
            return False

def visualize(scan, line_intersections, res = (0.05, 0.05), metric_size = (8, 8)):
    theta = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    pcl = np.empty((len(scan.ranges), 2))
    ranges = np.array(scan.ranges)
    ranges[np.isinf(ranges.copy())==True] = scan.range_max
    pcl[:,0] = np.multiply(ranges, np.cos(theta))
    pcl[:,1] = np.multiply(ranges, np.sin(theta))
    shape = np.round(np.array(metric_size)/res).astype(np.int)
    M = np.zeros(shape = shape)
    pcl_dig = np.zeros((len(pcl),2), dtype = np.int)
    
    pcl_dig[:,0] = (np.round(pcl[:,0]/res[0]) + shape[0]//2).astype(np.int)
    pcl_dig[:,1] = (np.round(pcl[:,1]/res[1]) + shape[1]//2).astype(np.int)
    
    pcl_dig = pcl_dig[(pcl_dig[:,0] < shape[0]) * (pcl_dig[:,0] >= 0) * (pcl_dig[:,1] < shape[1]) * (pcl_dig[:,1] >= 0)]
    
    if len(pcl_dig)>0:
        M[pcl_dig[:,0], pcl_dig[:,1]] = 0.3

    if len(line_intersections) > 0:
        line_intersections_dig = np.zeros((line_intersections.shape[0],2), dtype = np.int)
        
        line_intersections_dig[:,0] = (np.round(line_intersections[:,0]/res[0]) + shape[0]//2).astype(np.int)
        line_intersections_dig[:,1] = (np.round(line_intersections[:,1]/res[1]) + shape[1]//2).astype(np.int)
        
        line_intersections_dig = line_intersections_dig[(line_intersections_dig[:,0] < shape[0]) * (line_intersections_dig[:,0] >= 0) * (line_intersections_dig[:,1] < shape[1]) * (line_intersections_dig[:,1] >= 0)]
        
        if len(line_intersections_dig)>0:
            M[line_intersections_dig[:,0], line_intersections_dig[:,1]] = 1
    return M
        

            

