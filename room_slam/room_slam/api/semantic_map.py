import numpy as np
from sklearn.neighbors import NearestNeighbors
from room_slam.api.transformations import lines_msg2array, transform_scan_msg2array, landmarks_msg2array, transform2d, objects_msg2array, transform3d, exits_msg2array
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from room_slam.api.markers_generator import corners_marker, exits_marker, objects_marker, room_marker
from visualization_msgs.msg import MarkerArray
from scipy.stats import multivariate_normal
from shapely.geometry import Point, Polygon
from scipy.optimize import differential_evolution
import pickle
from skimage.draw import line as Line
from nav_msgs.msg import OccupancyGrid
from scipy.spatial.transform import Rotation
from laser_line_extraction.msg import LineSegmentList

import networkx as nx

class Objects():
    def __init__(self):
        self.poses = []
        self.P = []
        self.C = []

class Walls():
    def __init__(self):
        self.points_a = []
        self.points_b = []
        self.norms = []

class Room():
    def __init__(self):
        self.corners = [] # 2d array (n X 2) i.e. [[x1,y1], [x2, y2], ...]
        self.exits = [] # 3d array (n X 2 X 2) i.e. [[[x11,y11], [x12, y12]], [[x21,y21], [x22, y22]], ...]
        self.walls = Walls()
        self.objects = Objects()
        self.Mu = []
        self.Sigma = []
        self.params = None
        self.valid = False
        self.ogmap = OccupancyGridMap(origin = None, size = np.array([20,20]), shape = np.array([200,200]))


def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return np.array([x, y]), div

def _is_intesection_inside(intesection, A , B, th):
    da = np.linalg.norm(intesection - A)
    db = np.linalg.norm(intesection - B)
    ab = np.linalg.norm(B - A)
    if ab > da - th and ab > db - th:
        return True
    else:
        return False

def find_closest_line(line, lines):
    point_a, point_b = line
    mid = (point_a + point_b)/2
    points_a, points_b = lines
    mids = (points_a + points_b)/2
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(mids)
    _ , indices = nbrs.kneighbors([mid])
    return np.squeeze(indices)
    
def is_intersected(line1, line2):
    intersection, _ = line_intersection(line1, line2)
    result = _is_intesection_inside(intersection, line1[0] , line1[1], 0.5)
    return result

def in_prohibided_zone(point, line, normal):
    v1 = line[0] - point
    v2 = line[1] - point
    v = line[0] - line[1]
    if np.dot(v,v1) * np.dot(v,v2) < 0 and np.dot(normal,v1) > 0 and np.dot(normal,v2) > 0:
        return True
    else:
        return False

def same_line(AB1,AB2,threshold = 0.4):
    '''Check if lines are parrallel and if lines close to each other
    True - probably same line
    False - probably new line'''
    L1 = AB1[1] - AB1[0]
    L2 = AB2[1] - AB2[0]
    l1 = L1 / np.linalg.norm(L1)
    l2 = L2 / np.linalg.norm(L2)
    x1,y1 = l1
    x2,y2 = l2
    d = x1*y2 - x2*y1
    th = np.arcsin(d)
    if np.abs(th) * (180/np.pi) < 30:
        c1 = (AB1[1] + AB1[0])/2
        c2 = (AB2[1] + AB2[0])/2
        if np.linalg.norm(c2-c1) < threshold:
            return True
    return False

def line2normal(line, pose):
    v = line[0] - line[1]
    d = line[0] - pose[:2]
    normal_pos = np.array([v[1], -v[0]])
    if np.dot(normal_pos, d) > 0:
        return normal_pos
    else:
        return -normal_pos

def map_line(line, lines, normals , threshold = 0.8):
    closest_line_idx = find_closest_line(line, lines)
    closest_line = [lines[0][closest_line_idx], lines[1][closest_line_idx]]
    inter = []
    for ii in range(len(normals)):
        line_temp =  [lines[0][ii], lines[1][ii]]
        inter.append(is_intersected(line, line_temp))
    mid = (line[0] + line[1])/2 
    # closest_mid = (closest_line[0] + closest_line[1])/2 
    prohib = in_prohibided_zone(mid, closest_line, normals[closest_line_idx])
    if np.sum(inter) == 0 and prohib == False and not same_line(line,closest_line,threshold = threshold):
        return True
    else:
        return False

def line_likelihood(line, lines, normals):
    alpha1 = 0.25
    alpha2 = 0.25
    closest_line_idx = find_closest_line(line, lines)
    closest_line = [lines[0][closest_line_idx], lines[1][closest_line_idx]]
    a0 = (line[1][1] - line[0][1])/(line[1][0] - line[0][0])
    b0 = line[0][1] - line[0][0] * a0
    a1 = (closest_line[1][1] - closest_line[0][1])/(closest_line[1][0] - closest_line[0][0])
    b1 = closest_line[0][1] - closest_line[0][0] * a1
    L = np.exp(-alpha1*(a0-a1)**2 -alpha2*(b0-b1)**2)
    return L


def room_likelihood_opt(room_params, z , wa):
    mux, muy, theta, a, b = room_params
    f = 100/(a*b)
    prob = np.zeros(len(z)+len(wa))
    coords = [(mux + 0.5*a*np.cos(theta) + 0.5*b*np.sin(theta), muy + 0.5*a*np.sin(theta) + 0.5*b*np.cos(theta)),
              (mux + 0.5*a*np.cos(theta) - 0.5*b*np.sin(theta), muy + 0.5*a*np.sin(theta) - 0.5*b*np.cos(theta)),
              (mux - 0.5*a*np.cos(theta) - 0.5*b*np.sin(theta), muy - 0.5*a*np.sin(theta) - 0.5*b*np.cos(theta)),
              (mux - 0.5*a*np.cos(theta) + 0.5*b*np.sin(theta), muy - 0.5*a*np.sin(theta) + 0.5*b*np.cos(theta))]
    poly = Polygon(coords)
    for ii in range(len(z)):
        p = Point(z[ii,0], z[ii,1])
        if p.within(poly):
            prob[ii] = -1.1*f
        else:
            prob[ii] = 0
    # if len(wa) > 1:
    #     for jj in range(len(wa)):
    #         p = Point(wa[jj,0], wa[jj,1])
    #         if p.within(poly):
    #             prob[ii+jj] = -0.01*f
    #         else:
    #             prob[ii+jj] = 0

    return np.sum(prob)

def optimal_room_params(z,wa, old_params = None):
    min_room_size = 4.0
    max_room_size = 8.0
    if old_params is None:
        maxiter = 50
        popsize = 6
        theta0 = 0
        mux0 = (0*np.mean(z[:,0]) + 0.5*(np.max(z[:,0]) + np.min(z[:,0])))
        muy0 = (0*np.mean(z[:,1]) + 0.5*(np.max(z[:,1]) + np.min(z[:,1])))
        xx = (np.max(z[:,0]) - np.min(z[:,0]))/8
        yy = (np.max(z[:,1]) - np.min(z[:,1]))/8
        a0 = np.max(z[:,0]) - np.min(z[:,0])
        b0 = np.max(z[:,1]) - np.min(z[:,1])
        max_room_size = np.min(np.array([(a0**2 + b0**2)**0.5,6]))
        if min_room_size > max_room_size:
            min_room_size = max_room_size - 0.01

        bounds = [[mux0 - xx, mux0 + xx],
                [muy0 - yy, muy0 + yy],
                [theta0 - np.pi/4, theta0 + np.pi/4],
                [min_room_size, max_room_size],
                [min_room_size,max_room_size]]
    

    
        L = lambda x: room_likelihood_opt(x, z , wa)
        result = differential_evolution(L, bounds,maxiter=maxiter,popsize=popsize,tol=0.001)
        print('Center of advised room: ' , result.x[0:2])
        print('Orientation: ' , np.round(180*result.x[2]/np.pi,decimals=3))
        print('Size of the room: ' , result.x[3:])

        return result.x
    else:
        return old_params
    


def multivariate_uniform(pose, room_params):
    mux, muy, theta, a, b = room_params
    f = 1/(a*b)
    coords = [(mux + 0.5*a*np.cos(theta) + 0.5*b*np.sin(theta), muy + 0.5*a*np.sin(theta) + 0.5*b*np.cos(theta)),
              (mux + 0.5*a*np.cos(theta) - 0.5*b*np.sin(theta), muy + 0.5*a*np.sin(theta) - 0.5*b*np.cos(theta)),
              (mux - 0.5*a*np.cos(theta) - 0.5*b*np.sin(theta), muy - 0.5*a*np.sin(theta) - 0.5*b*np.cos(theta)),
              (mux - 0.5*a*np.cos(theta) + 0.5*b*np.sin(theta), muy - 0.5*a*np.sin(theta) + 0.5*b*np.cos(theta))]
    poly = Polygon(coords)
    p = Point(pose[0], pose[1])
    if p.within(poly):
        return 1
    else:
        return 0



class OccupancyGridMap():
    def __init__(self, origin, size, shape):
        self.shape = shape
        self.size = size
        self.map = 127*np.ones(shape, dtype=np.uint8)
        self.res = self.size/self.shape
        self.origin = origin

def update_ogmap(scan, pose, map):
    pose = pose - map.origin
    pose_dig = np.array([(pose[0])/map.res[0] + map.shape[0]//2 , (pose[1])/map.res[1] + map.shape[1]//2], dtype=np.int)
    theta = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    pcl = np.empty((len(scan.ranges), 2))
    ranges = np.array(scan.ranges)
    ranges[np.isinf(ranges.copy())==True] = scan.range_max
    pcl[:,0] = np.multiply(ranges, np.cos(theta))
    pcl[:,1] = np.multiply(ranges, np.sin(theta))
    pcl[:,0] = pcl[:,0] + 0.17
    pcl[:,1] = -pcl[:,1]
    pcl_transofrmed = transform2d(pcl, pose)
    pcl_dig = np.zeros((len(pcl_transofrmed),2), dtype = np.int)
    pcl_dig[:,0] = (np.round(pcl_transofrmed[:,0]/map.res[0]) + map.shape[0]//2).astype(np.int)
    pcl_dig[:,1] = (np.round(pcl_transofrmed[:,1]/map.res[1]) + map.shape[1]//2).astype(np.int)
    for ii in range(len(pcl_dig)):
        if pcl_dig[ii,0] < map.shape[0] and pcl_dig[ii,0] >= 0 and pcl_dig[ii,1] < map.shape[1] and pcl_dig[ii,1] >= 0:
            rr, cc = Line(pose_dig[0], pose_dig[1], pcl_dig[ii,0], pcl_dig[ii,1])
            map.map[rr, cc] = 0
            map.map[pcl_dig[ii,0], pcl_dig[ii,1]] = 255
    return map

def get_occupancy(map, pose):
    pose_dig = np.array([pose[0]/map.res[0] + map.shape[0]//2, pose[1]/map.res[1] + map.shape[1]//2], dtype=np.int)
    return map.map[pose_dig[0], pose_dig[1]]



class SemanticMap():
    def __init__(self, frame_id):
        self.rooms = []
        self.current_room_id = -1
        self.n_rooms  =  0
        self.dth = 0.55
        self.sigma = 1.5
        self.frame_id = frame_id
        self.G = nx.Graph()
        self.G.add_node(0)
        self.G_pose_samples = {(0,0): [np.array([0,0], dtype=np.float)]}
        self.Laplacian = np.array(1)

    def new_node(self, old_node, pose):
        
        d = np.linalg.norm(self.rooms[old_node].Mu - pose[:2])
        self.G.add_node(self.n_rooms)
        self.G.add_edge(old_node, self.n_rooms, weight = d)

        Laplacian_temp = np.zeros((self.n_rooms, self.n_rooms))
        Laplacian_temp[:-1, :-1] = self.Laplacian
        Laplacian_temp[old_node, -1] = 1
        Laplacian_temp[-1, old_node] = Laplacian_temp[old_node, -1]
        self.Laplacian = Laplacian_temp
        self.G_pose_samples[(self.n_rooms-1, old_node)] = [pose[:2]]
        self.G_pose_samples[(old_node, self.n_rooms-1)] = self.G_pose_samples[(self.n_rooms-1, old_node)]
    

    def new_edge(self, node_a, node_b, pose):
        if self.Laplacian[node_a, node_b] == 0:
            self.G_pose_samples[(node_a, node_b)] = []
            d = np.linalg.norm(self.rooms[node_a].Mu - self.rooms[node_b].Mu)
            self.G.add_edge(node_a, node_b, weight = d)
        self.Laplacian[node_a, node_b] = 1
        self.Laplacian[node_b, node_a] = self.Laplacian[node_a, node_b]
        self.G_pose_samples[(node_a, node_b)].append(pose[:2])
        self.G_pose_samples[(node_b, node_a)] = self.G_pose_samples[(node_a, node_b)]

    def visualize_graph(self):
        poses = np.concatenate(list(self.G_pose_samples.values()))
        output_marker =  corners_marker(
            id = self.current_room_id,
            poses = poses,
            ns = 'edges_map',
            color = (1,1,0),
            frame_id = self.frame_id)
        return output_marker
        

    ''' Yet to be completed'''
    def save_map(self, path):
        with open(path, 'wb') as output:
            pickle.dump([self.rooms,self.G_pose_samples,self.Laplacian] , output, pickle.HIGHEST_PROTOCOL)
    
    def load_map(self, path):
        with open(path, 'rb') as input:
            self.rooms,self.G_pose_samples,self.Laplacian = pickle.load(input)
            self.n_rooms = len(self.rooms)
            self.current_room_id = -1

    def _update_ogmap(self, scan, pose):
        self.rooms[self.current_room_id].ogmap = update_ogmap(scan, pose, self.rooms[self.current_room_id].ogmap)

    def likelihood_room_ogmap(self, scan, pose, global_localization = False):
        if global_localization:
            room = np.argmax(self.likelihood_rooms(pose))
        else:
            room = self.current_room_id
        oc = get_occupancy(self.rooms[room].ogmap, pose)
        if oc == 255 or oc == 127:
            return 0
        elif oc == 0:
            return 1
    
    def visualize_ogmap(self):
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.rooms[self.current_room_id].ogmap.res[0]
        map_msg.info.width = self.rooms[self.current_room_id].ogmap.shape[0]
        map_msg.info.height = self.rooms[self.current_room_id].ogmap.shape[1]
        origin = Pose()
        origin.position.x = self.rooms[self.current_room_id].ogmap.origin[0]
        origin.position.y = self.rooms[self.current_room_id].ogmap.origin[1]
        r = Rotation.from_euler('z',self.rooms[self.current_room_id].ogmap.origin[2], degrees=False)
        origin.orientation.x =r.as_quat()[0]
        origin.orientation.y =r.as_quat()[1]
        origin.orientation.z =r.as_quat()[2]
        origin.orientation.w =r.as_quat()[3]
        map_msg.info.origin = origin
        data = self.rooms[self.current_room_id].ogmap.map.copy()
        data[data == 127] = -1
        data[data == 255] = 100
        map_msg.data = data.reshape((-1))
        
        return map_msg

    def is_rooms_available(self):
        if self.n_rooms > 0:
            valid_rooms = 0
            for ii in range(self.n_rooms):
                valid_rooms = valid_rooms + self.rooms[ii].valid
            if len(self.rooms[self.current_room_id].Mu) > 0 and valid_rooms == self.n_rooms:
                return True
            else:
                return False
        else:
            return False

    def is_exits_available(self):
        try:
            if self.n_rooms > 0:
                if len(self.rooms[self.current_room_id].exits) > 0:
                    return True
                else:
                    return False
            else:
                return False
        except:
            print('list index out of range.')
            return False
    def is_walls_available(self):
        try:
            if self.n_rooms > 0:
                if len(self.rooms[self.current_room_id].walls.norms) > 0:
                    return True
                else:
                    return False
            else:
                return False
        except:
            print('list index out of range.')
            return False

    def is_corners_available(self):
        try:
            if self.n_rooms > 0:
                if len(self.rooms[self.current_room_id].corners) > 0:
                    return True
                else:
                    return False
            else:
                return False
        except:
            print('list index out of range.')
            return False
    def is_objects_available(self):
        try:
            if self.n_rooms > 0:
                if len(self.rooms[self.current_room_id].objects.poses) > 0:
                    return True
                else:
                    return False
            else:
                return False
        except:
            print('list index out of range.')
            return False
    def rooms_evidance(self, pose):
        Lu = np.zeros(self.n_rooms)
        Lg = np.zeros(self.n_rooms)
        for ii in range(self.n_rooms):
            Lu[ii] = multivariate_uniform(pose, room_params = self.rooms[ii].params)
            Lg[ii] = multivariate_normal.pdf(pose[:2], mean=self.rooms[ii].Mu, cov=self.rooms[ii].Sigma)
        return 0.7*Lu + 0.3*Lg

    def new_room(self, relative_pose, corners_msg = None, objects_msg = None, exits_msg = None , walls_msg = None):
        
        old_room = self.current_room_id
        self.current_room_id = self.n_rooms
        self.n_rooms = self.n_rooms + 1
        
        self.rooms.append(Room())
        self.rooms[self.current_room_id].ogmap.origin = relative_pose
        if corners_msg is not None:
            corners_array = landmarks_msg2array(corners_msg)
            
            corners_array_trans = transform2d(corners_array, relative_pose)
            self.rooms[self.current_room_id].corners = corners_array_trans

        if exits_msg is not None:
            exits_array = exits_msg2array(exits_msg)
            exits_array_trans = transform2d(exits_array, relative_pose)
            self.rooms[self.current_room_id].exits = exits_array_trans

        if objects_msg is not None:
            objects_array = objects_msg2array(objects_msg)
            objects_array_trans = transform3d(objects_array, relative_pose)
            self.rooms[self.current_room_id].objects.poses = objects_array_trans
            for obj in objects_msg.objects:
                self.rooms[self.current_room_id].objects.P.append(obj.probability)
                self.rooms[self.current_room_id].objects.C.append(obj.cls)

        if walls_msg is not None:
            lines_array_a, lines_array_b = lines_msg2array(walls_msg)
            lines_array_a_trans = transform2d(lines_array_a, relative_pose)
            lines_array_b_trans = transform2d(lines_array_b, relative_pose)
            self.rooms[self.current_room_id].walls.points_a = lines_array_a_trans
            self.rooms[self.current_room_id].walls.points_b = lines_array_b_trans
            for ii in range(lines_array_a_trans.shape[0]):
                self.rooms[self.current_room_id].walls.norms.append(line2normal([lines_array_a_trans[ii,:], lines_array_b_trans[ii,:]], relative_pose))
        if old_room == -1:
            self.rooms[self.current_room_id].Mu = relative_pose[:2]
        self.update_room_pdf()
        self.new_node(old_room, relative_pose)


    def advise_room(self, pose , max_d = 4):
        
        try: n_objects = sum(np.linalg.norm(np.array(self.rooms[self.current_room_id].objects.poses).T - np.array(pose[:2]),axis = 1) < max_d)
        except: n_objects = 0
        try: n_corners = sum(np.linalg.norm(self.rooms[self.current_room_id].corners - np.array(pose[:2]),axis = 1) < max_d)
        except: n_corners = 0
        try: n_exits = sum(np.linalg.norm(self.rooms[self.current_room_id].exits - np.array(pose[:2]),axis = 1) < max_d)/2
        except: n_exits = 0
        try: L_arr = self.rooms_evidance(pose)
        except: L_arr = np.ones(self.n_rooms)
        
        
        L = np.max(L_arr)
    # ---------------------------Condition for new room---------------------------------
        
        if L < 0.007 and n_corners + n_objects + n_exits > 2:
            
            new_room_advice = True
            move_to_room_advice = None
            update_map_advice = True

        else:
            update_map_advice = True
            new_room_advice = False
            try:
                L = self.likelihood_rooms(pose)
                move_to_room_advice = np.argmax(L)
            except:
                move_to_room_advice = self.current_room_id
    

        return new_room_advice, move_to_room_advice, update_map_advice


    def likelihood_rooms(self, pose):
        L = self.rooms_evidance(pose)
        if np.sum(L) > 0:
            L = L/np.sum(L)
        return L


    def update_room_pdf(self):
        room_data = []
        wall_data = []
        if len(self.rooms[self.current_room_id].corners) > 0:
            
            room_data.append(self.rooms[self.current_room_id].corners)
            
            
        if len(self.rooms[self.current_room_id].exits) > 0:
            room_data.append(self.rooms[self.current_room_id].exits)
            
        # if len(self.rooms[self.current_room_id].walls.norms) > 0:
        #     wa = []
        #     wb = []
        #     for (a,b) in zip(self.rooms[self.current_room_id].walls.points_a,self.rooms[self.current_room_id].walls.points_b):
        #         ap = np.array(a)
        #         bp = np.array(b)
        #         p = (ap+bp)/2
        #         if np.linalg.norm(p) < 3:
        #             wa.append(a)
        #             wb.append(b)
        #     if wa:
        #         room_data.append(wa)
        #         room_data.append(wb)
        if len(self.rooms[self.current_room_id].objects.poses) > 0:
            if self.rooms[self.current_room_id].objects.poses.shape == (3,):
                room_data.append([self.rooms[self.current_room_id].objects.poses[:2]])
                
            else:
                room_data.append(self.rooms[self.current_room_id].objects.poses[:,:2])
        
        try: room_data = np.concatenate(room_data, axis = 0)
        except: room_data = []

        if len(room_data) > 1:
            
            if len(wall_data) > 1:
                wall_data = np.concatenate(wall_data, axis = 0)
            else: wall_data = np.array([])
            self.rooms[self.current_room_id].Mu = np.mean(room_data, axis = 0)
            self.rooms[self.current_room_id].Sigma = np.cov(room_data.T)
            if self.rooms[self.current_room_id].params is None:
                
                self.rooms[self.current_room_id].params = optimal_room_params(room_data , wall_data)
            else:
                self.rooms[self.current_room_id].params = optimal_room_params(room_data,wall_data, self.rooms[self.current_room_id].params)
            
           
        if len(room_data) > 3:
            self.rooms[self.current_room_id].valid = True

        


    def update_room_corners(self, relative_pose, corners_msg):
        '''
        To do:
        1) Filter corners
        '''
        update_room_ind = False
        corners_array = landmarks_msg2array(corners_msg)
        corners_array_trans = transform2d(corners_array, relative_pose)
        if len(self.rooms[self.current_room_id].corners) == 0:
            self.rooms[self.current_room_id].corners = corners_array_trans
            update_room_ind = True
        else:
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(self.rooms[self.current_room_id].corners)
            for ii in range(corners_array_trans.shape[0]):
                _ , indices = nbrs.kneighbors([corners_array_trans[ii,:]])
                indices = np.squeeze(indices)
                closest_pose = self.rooms[self.current_room_id].corners[indices]
                pose_dist = np.linalg.norm(corners_array_trans[ii,:] - closest_pose)
                if pose_dist > self.dth:
                    self.rooms[self.current_room_id].corners = np.append(self.rooms[self.current_room_id].corners, [corners_array_trans[ii,:]], axis = 0) 
                    update_room_ind = True
        if update_room_ind:
            self.update_room_pdf()

    def update_room_exits(self, relative_pose, exits_msg, scan = None):
        '''
        To do:
        1) Filter exits
        '''
        update_room_ind = False
        exits_array = exits_msg2array(exits_msg)
        exits_array_trans = transform2d(exits_array, relative_pose)
        if len(self.rooms[self.current_room_id].exits) == 0:
            self.rooms[self.current_room_id].exits = exits_array_trans
            update_room_ind = True
        else:
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(self.rooms[self.current_room_id].exits)
            for ii in range(0, exits_array_trans.shape[0]-1, 2):
                A = exits_array_trans[ii,:]
                B = exits_array_trans[ii+1,:]
                _ , indices1 = nbrs.kneighbors([A])
                _ , indices2 = nbrs.kneighbors([B])
                indices1 = np.squeeze(indices1)
                indices2 = np.squeeze(indices2)
                closest_pose1 = self.rooms[self.current_room_id].exits[indices1]
                closest_pose2 = self.rooms[self.current_room_id].exits[indices2]
                pose_dist1 = np.linalg.norm(A - closest_pose1)
                pose_dist2 = np.linalg.norm(B - closest_pose2)
                if pose_dist1 > 2*self.dth and pose_dist2 > 2*self.dth:
                    update_room_ind = True
                    if scan is None:
                        self.rooms[self.current_room_id].exits = np.append(self.rooms[self.current_room_id].exits, [A], axis = 0)
                        self.rooms[self.current_room_id].exits = np.append(self.rooms[self.current_room_id].exits, [B], axis = 0) 
                        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(self.rooms[self.current_room_id].exits)
                    else:
                        C = (exits_array[ii,:] + exits_array[ii+1,:])/2
                        theta = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
                        pcl = np.empty((len(scan.ranges), 2))
                        ranges = np.array(scan.ranges)
                        ranges[np.isinf(ranges.copy())==True] = scan.range_max
                        pcl[:,0] = np.multiply(ranges, np.cos(theta))
                        pcl[:,1] = np.multiply(ranges, np.sin(theta))
                        pcl[:,0] = pcl[:,0] + 0.17
                        pcl[:,1] = -pcl[:,1]
                        d = np.linalg.norm(pcl - C, axis = 1)
                        #d = np.linalg.norm(transform_scan_msg2array(scan, relative_pose) - C, axis = 1)
                        if np.min(d) > 0.3:
                            self.rooms[self.current_room_id].exits = np.append(self.rooms[self.current_room_id].exits, [A], axis = 0)
                            self.rooms[self.current_room_id].exits = np.append(self.rooms[self.current_room_id].exits, [B], axis = 0)
                            nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(self.rooms[self.current_room_id].exits)
        if update_room_ind:
            self.update_room_pdf() 

    def update_room_walls(self, relative_pose, lines_msg):
        update_room_ind = False
        lines_array_a, lines_array_b = lines_msg2array(lines_msg)
        lines_array_a_trans = transform2d(lines_array_a, relative_pose)
        lines_array_b_trans = transform2d(lines_array_b, relative_pose)
        if len(self.rooms[self.current_room_id].walls.norms) == 0:
            self.rooms[self.current_room_id].walls.points_a = lines_array_a_trans
            self.rooms[self.current_room_id].walls.points_b = lines_array_b_trans
            for ii in range(lines_array_a_trans.shape[0]):
                self.rooms[self.current_room_id].walls.norms.append(line2normal([lines_array_a_trans[ii,:], lines_array_b_trans[ii,:]], relative_pose))
            update_room_ind = True
        else:
            for ii in range(lines_array_a_trans.shape[0]):
                A = lines_array_a_trans[ii,:]
                B = lines_array_b_trans[ii,:]
                if map_line([A, B], [self.rooms[self.current_room_id].walls.points_a, self.rooms[self.current_room_id].walls.points_b], self.rooms[self.current_room_id].walls.norms ):
                    update_room_ind = True
                    self.rooms[self.current_room_id].walls.points_a = np.append(self.rooms[self.current_room_id].walls.points_a, [A], axis = 0)
                    self.rooms[self.current_room_id].walls.points_b = np.append(self.rooms[self.current_room_id].walls.points_b, [B], axis = 0)
                    self.rooms[self.current_room_id].walls.norms.append(line2normal([A, B], relative_pose))
                


    def visualize_walls(self):
        poses = []
        for ii in range(len(self.rooms[self.current_room_id].walls.points_a)):
            try:
                poses.append([self.rooms[self.current_room_id].walls.points_a[ii,:]])
                poses.append([self.rooms[self.current_room_id].walls.points_b[ii,:]])
            except:
                print("Could not append wall landmark, tried too soon.")
        poses = np.concatenate(poses, axis = 0)
        output_marker =  exits_marker(
            id = self.current_room_id,
            exits = poses,
            ns = 'walls_map',
            color = (0,0,1),
            frame_id = self.frame_id)
        return output_marker



    def update_room_objects(self, relative_pose, objects_msg):
        update_room_ind = False
        objects_array = objects_msg2array(objects_msg)
        objects_array_trans = transform3d(objects_array, relative_pose)
        if len(self.rooms[self.current_room_id].objects.poses) == 0:
            update_room_ind = True
            self.rooms[self.current_room_id].objects.poses = objects_array_trans
            for obj in objects_msg.objects:
                self.rooms[self.current_room_id].objects.P.append(obj.probability)
                self.rooms[self.current_room_id].objects.C.append(obj.cls)
        else:

            if self.rooms[self.current_room_id].objects.poses.shape == (3,):
                self.rooms[self.current_room_id].objects.poses = np.array([self.rooms[self.current_room_id].objects.poses])
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(self.rooms[self.current_room_id].objects.poses)
            
            if objects_array_trans.shape == (3,):
                objects_array_trans = np.array([objects_array_trans])
            for ii in range(objects_array_trans.shape[0]):
                _ , indices = nbrs.kneighbors([objects_array_trans[ii,:]])
                indices = np.squeeze(indices)
                closest_pose = self.rooms[self.current_room_id].objects.poses[indices]
                p_temp = objects_msg.objects[ii].probability
                c_temp = objects_msg.objects[ii].cls
                closest_cls = self.rooms[self.current_room_id].objects.C[indices]
                pose_dist = np.linalg.norm(objects_array_trans[ii,:] - closest_pose)
                if pose_dist > 1.5*self.dth and c_temp != closest_cls:
                    update_room_ind = True
                    self.rooms[self.current_room_id].objects.poses = np.append(self.rooms[self.current_room_id].objects.poses, [objects_array_trans[ii,:]], axis = 0)
                    self.rooms[self.current_room_id].objects.P.append(objects_msg.objects[ii].probability)
                    self.rooms[self.current_room_id].objects.C.append(objects_msg.objects[ii].cls)
        if update_room_ind:
            self.update_room_pdf()

    def likelihood_room_corners(self, relative_pose, corners_msg, velocity = None, global_localization = False):
        if global_localization and self.n_rooms > 1:
            room = np.argmax(self.likelihood_rooms(relative_pose))
        else:
            room = self.current_room_id
        if velocity is None:
            sig = 1.1*self.sigma
        else:
            sig = self.sigma/(velocity*10)
        corners_array = landmarks_msg2array(corners_msg)
        corners_array_trans = transform2d(corners_array, relative_pose)
        if len(self.rooms[room].corners) == 0:
            return 0
        else:
            p = np.zeros(corners_array_trans.shape[0])
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(self.rooms[room].corners)
            for ii in range(corners_array_trans.shape[0]):
                _ , indices = nbrs.kneighbors([corners_array_trans[ii,:]])
                indices = np.squeeze(indices)
                closest_pose = self.rooms[room].corners[indices]
                pose_dist = np.linalg.norm(corners_array_trans[ii,:] - closest_pose)
                p[ii] = np.exp(-sig*(pose_dist**2))
            return np.mean(p)

    def likelihood_room_exits(self, relative_pose, exits_msg, velocity = None, global_localization = False):
        if global_localization and self.n_rooms > 1:
            room = np.argmax(self.likelihood_rooms(relative_pose))
        else:
            room = self.current_room_id
        if velocity is None:
            sig = self.sigma/2
        else:
            sig = self.sigma/(20*velocity)
        exits_array = exits_msg2array(exits_msg)
        exits_array_trans = transform2d(exits_array, relative_pose)
        if len(self.rooms[room].exits) == 0:
            return 0
        else:
            p = np.zeros(exits_array_trans.shape[0])
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(self.rooms[room].exits)
            for ii in range(exits_array_trans.shape[0]):
                _ , indices = nbrs.kneighbors([exits_array_trans[ii,:]])
                indices = np.squeeze(indices)
                closest_pose = self.rooms[room].exits[indices]
                pose_dist = np.linalg.norm(exits_array_trans[ii,:] - closest_pose)
                p[ii] = np.exp(-sig*(pose_dist**2))
            return np.mean(p)

    def likelihood_walls(self, relative_pose, lines_msg, velocity = None, global_localization = False):
        if global_localization and self.n_rooms > 1:
            room = np.argmax(self.likelihood_rooms(relative_pose))
        else:
            room = self.current_room_id
        if velocity is None:
            sig = self.sigma
        else:
            sig = self.sigma/(velocity*10)
        lines_array_a, lines_array_b = lines_msg2array(lines_msg)
        lines_array_a_trans = transform2d(lines_array_a, relative_pose)
        lines_array_b_trans = transform2d(lines_array_b, relative_pose)
        if len(self.rooms[room].walls.norms) == 0:
            return 0
        else:
            p = np.zeros(lines_array_a_trans.shape[0])
            for ii in range(lines_array_a_trans.shape[0]):
                line_temp = [lines_array_a_trans[ii,:], lines_array_b_trans[ii,:]]
                p[ii] = line_likelihood(line_temp, [self.rooms[self.current_room_id].walls.points_a, self.rooms[self.current_room_id].walls.points_b], self.rooms[self.current_room_id].walls.norms)
            return np.mean(p)

    def likelihood_room_objects(self, relative_pose, objects_msg, velocity = None, global_localization = False):
        if velocity is None:
            sig = self.sigma
        else:
            sig = self.sigma/(velocity*10)
        objects_array = objects_msg2array(objects_msg)
        objects_array_trans = transform3d(objects_array, relative_pose)
        if global_localization:
            room = np.argmax(self.likelihood_rooms(relative_pose))
        else:
            room = self.current_room_id
        if len(self.rooms[room].objects.poses) == 0:
            return 0
        else:
            p = np.zeros(objects_array_trans.shape[0])
            if self.rooms[room].objects.poses.shape == (3,):
                nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array([self.rooms[room].objects.poses]))
            else:
                nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(self.rooms[room].objects.poses)
            if objects_array_trans.shape == (3,):
                objects_array_trans = np.array([objects_array_trans])
   
            for ii in range(objects_array_trans.shape[0]):
                _ , indices = nbrs.kneighbors([objects_array_trans[ii,:]])
                indices = np.squeeze(indices)
                closest_pose = self.rooms[room].objects.poses[indices]
                pose_dist = np.linalg.norm(objects_array_trans[ii,:] - closest_pose)
                p_temp = objects_msg.objects[ii].probability
                c_temp = objects_msg.objects[ii].cls
                closest_p = self.rooms[room].objects.P[indices]
                closest_cls = self.rooms[room].objects.C[indices]
                if c_temp == closest_cls:
                    p_dist = closest_p * p_temp
                else:
                    p_dist = ((1 - closest_p)/80) * ((1 - p_temp)/80)
                p[ii] = p_dist * np.exp(-sig* (pose_dist**2))
            return np.mean(p)

    def visualize_corners(self, visualize_all_rooms = False):
        if visualize_all_rooms:
            poses = []
            for ii in range(self.n_rooms):
                try:
                    poses.append(self.rooms[ii].corners)
                except:
                    print('Could not append corner landmark, tried too soon.')
            poses = np.concatenate(poses, axis = 0)
        else:
            poses = self.rooms[self.current_room_id].corners
        output_marker =  corners_marker(
            id = self.current_room_id,
            poses = poses,
            ns = 'corners_map',
            color = (0,1,0),
            frame_id = self.frame_id)
        return output_marker

    def visualize_exits(self, visualize_all_rooms = False):
        if visualize_all_rooms:
            poses = []
            for ii in range(self.n_rooms):
                try:
                    poses.append(self.rooms[ii].exits)
                except:
                    print('Could not append exit landmark, tried too soon.')
            poses = np.concatenate(poses, axis = 0)
        else:
            poses = self.rooms[self.current_room_id].exits
        output_marker =  exits_marker(
            id = self.current_room_id,
            exits = poses,
            ns = 'exits_map',
            color = (1,1,0),
            frame_id = self.frame_id)
        return output_marker

    def visualize_objects(self, visualize_all_rooms = False):
        if visualize_all_rooms:
            objects = []
            for ii in range(self.n_rooms):
                for jj in range(len(self.rooms[ii].objects.poses)):
                    objects.append(self.rooms[ii].objects[jj])
        else:
            objects = self.rooms[self.current_room_id].objects
        output_marker = objects_marker(objects = objects, frame_id = self.frame_id, color = (1,1,1))
        return output_marker

    def visualize_rooms(self, visualize_all_rooms = False):
        #self.update_room_pdf()
        list_marker = MarkerArray()
        list_marker.markers = []
        if visualize_all_rooms:
            for ii in range(self.n_rooms):
                mux, muy, theta, a, b = self.rooms[ii].params
                mu = [mux, muy]
                room = [mu[0], mu[1], theta,a,b]
                if ii == self.current_room_id:
                    marker_temp = room_marker(room = room, frame_id = self.frame_id, color = (1,0,0), ns = 'rooms', id = ii)
                else:
                    marker_temp = room_marker(room = room, frame_id = self.frame_id, color = (1,1,1), ns = 'rooms', id = ii)
                list_marker.markers.append(marker_temp)

        else:
            mux, muy, theta, a, b = self.rooms[self.current_room_id].params
            mu = [mux, muy]
            room = [mu[0], mu[1], theta,a,b]
            marker_temp = room_marker(room = room, frame_id = self.frame_id, color = (1,1,1), ns = 'rooms', id = self.current_room_id)
            list_marker.markers.append(marker_temp)
        return list_marker
            






    

