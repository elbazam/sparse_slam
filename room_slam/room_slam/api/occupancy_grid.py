#!/usr/bin/env python3

import numpy as np
import cv2

from skimage.draw import line
from scipy.optimize import differential_evolution
from scipy.stats import multivariate_normal

def EuclideanTransform(current_map, origin_map, warp_matrix = None):
    current_map_map = (current_map.map*255).astype(np.uint8)
    origin_map_map = (origin_map.map*255).astype(np.uint8)
    
    # Find the homography matrix. 
    if warp_matrix is None:
        warp_matrix = np.eye(2, 3, dtype=np.float32)
    warp_mode = cv2.MOTION_EUCLIDEAN
    sz = current_map_map.shape
    number_of_iterations = 300
    termination_eps = 1e-10
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations,  termination_eps)
    (cc, warp_matrix) = cv2.findTransformECC(origin_map_map, current_map_map,warp_matrix, warp_mode, criteria,None,1)
    inv_warp = np.linalg.inv(warp_matrix[:,:2])
    pos = inv_warp.dot(origin_map.map_dig_center - warp_matrix[:,2])
    alpha = np.arctan2(inv_warp[0,1],inv_warp[0,0])
    return np.array([pos[0], pos[1], alpha])


def scan2obs(scan, n = None, inf_range = None):
    if inf_range is None:
        inf_range = scan.range_max
    theta = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    z = np.empty((len(scan.ranges), 2))
    ranges = np.array(scan.ranges)
    ranges[np.isinf(ranges)==True] = inf_range
    z[:,0] = np.multiply(ranges, np.cos(theta))
    z[:,1] = np.multiply(ranges, np.sin(theta))
    if n is not None:
        idxs = np.linspace(0,len(z)-1,n, dtype=np.int)
        z = z[idxs]
    return z

def scan2map(scan, map_res, map_size, n = None, inf_range = None):
    map_shape = np.round(np.array(map_size)/np.array(map_res)).astype(np.int)
    map_dig_center = (map_shape[0]//2,map_shape[1]//2)
    map = 0.5 * np.ones((map_shape[0], map_shape[1]))
    z = scan2obs(scan, n, inf_range)
    z_dig = np.zeros(z.shape, dtype = np.int)
    for ii in range(2):
        z_dig[:,ii] = (np.round(z[:,ii]/map_res[ii]) + map_dig_center[ii]).astype(np.int)
    for ii in range(len(z_dig)):
        empty_obs = np.array(line(map_dig_center[0], map_dig_center[1],z_dig[ii,0], z_dig[ii,1] )).T
        empty_obs = empty_obs[(empty_obs[:,0] < map_shape[0]) * (empty_obs[:,0] >= 0) * (empty_obs[:,1] < map_shape[1]) * (empty_obs[:,1] >= 0)]
        if len(empty_obs)>0:
            map[empty_obs[:,0], empty_obs[:,1]] = 0

    z_dig = z_dig[(z_dig[:,0] < map_shape[0]) * (z_dig[:,0] >= 0) * (z_dig[:,1] < map_shape[1]) * (z_dig[:,1] >= 0)]
    if len(z_dig)>0:
        map[z_dig[:,0], z_dig[:,1]] = 1
    return map

def update_map(map, scan, x):
    theta = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    dtheta = (map.x0[2] - x[2]) % (2*np.pi)
    dx = map.x0[:2] - x[:2]
    b = np.round(dtheta/scan.angle_increment).astype(np.int)
    ranges = np.roll(np.array(scan.ranges), -b)
    ranges[np.isinf(ranges)==True] = map.inf_range
    z = np.empty((len(scan.ranges), 2))
    z[:,0] = np.multiply(ranges, np.cos(theta)) + dx[0]
    z[:,1] = np.multiply(ranges, np.sin(theta)) + dx[1]
    if map.n is not None:
        idxs = np.linspace(0,len(z)-1,map.n, dtype=np.int)
        z = z[idxs]
    
    z_dig = np.zeros(z.shape, dtype = np.int)
    for ii in range(2):
        z_dig[:,ii] = (np.round(z[:,ii]/map.map_res[ii]) + map.map_dig_center[ii]).astype(np.int)
    for ii in range(len(z_dig)):
        empty_obs = line(map.map_dig_center[0], map.map_dig_center[1],z_dig[ii,0], z_dig[ii,1] )
        empty_obs = empty_obs[(empty_obs[:,0] < map.map_shape[0]) * (empty_obs[:,0] >= 0) * (empty_obs[:,1] < map.map_shape[1]) * (empty_obs[:,1] >= 0)]
        if len(empty_obs)>0:
            map.map[empty_obs[:,0], empty_obs[:,1]] = map.map[empty_obs[:,0], empty_obs[:,1]] * 0.5
    z_dig = z_dig[(z_dig[:,0] < map.map_shape[0]) * (z_dig[:,0] >= 0) * (z_dig[:,1] < map.map_shape[1]) * (z_dig[:,1] >= 0)]
    if len(z_dig)>0:
        map.map[z_dig[:,0], z_dig[:,1]] = 1
    return map

def likelihood(x, x0, P0, scan ,map):
    theta = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    dtheta = (map.x0[2] - x[2]) % (2*np.pi)
    dx = map.x0[:2] - x[:2]
    b = np.round(dtheta/scan.angle_increment).astype(np.int)
    ranges = np.roll(np.array(scan.ranges), -b)
    ranges[np.isinf(ranges)==True] = map.inf_range
    z = np.empty((len(scan.ranges), 2))
    z[:,0] = np.multiply(ranges, np.cos(theta)) + dx[0]
    z[:,1] = np.multiply(ranges, np.sin(theta)) + dx[1]
    if map.n is not None:
        idxs = np.linspace(0,len(z)-1,map.n, dtype=np.int)
        z = z[idxs]
    z_dig = np.zeros(z.shape, dtype = np.int)
    for ii in range(2):
        z_dig[:,ii] = (np.round(z[:,ii]/map.map_res[ii]) + map.map_dig_center[ii]).astype(np.int)
    z_dig = z_dig[(z_dig[:,0] < map.map_shape[0]) * (z_dig[:,0] >= 0) * (z_dig[:,1] < map.map_shape[1]) * (z_dig[:,1] >= 0)]
    Lmap = map.likelihood_map()
    L2 = multivariate_normal.pdf(x, mean = x0, cov = P0)
    #L_pos = 1-Lmap[map.map_dig_center[0],map.map_dig_center[1]] 
    L = np.sum(Lmap[z_dig[:,0], z_dig[:,1]])
    return L*L2

def GlobalLocalization(x,P, scan ,map, maxiter, popsize, tol):
    func = lambda X: -likelihood(X, x, P, scan, map)
    bounds = [(x[0] - 2*np.sqrt(P[0,0]), x[0] + 2*np.sqrt(P[0,0])),
              (x[1] - 2*np.sqrt(P[1,1]), x[1] + 2*np.sqrt(P[1,1])),
              (x[2] - 2*np.sqrt(P[2,2]), x[2] + 2*np.sqrt(P[2,2]))]
    result = differential_evolution(func, bounds,maxiter=maxiter,popsize=popsize,tol=tol)
    return result.x

class OccupancyGridMap():
    def __init__(self, x0, P0, scan, map_res = (0.1, 0.1), map_size = (4, 4), n = None, inf_range = None, blur_kernel = (7,7)):
        self.x0 = x0
        self.P0 = P0
        self.map_res = map_res
        self.map_size = map_size
        self.n = n
        self.inf_range = inf_range
        self.map = scan2map(scan, map_res, map_size, n, inf_range)
        self.map_shape = np.round(np.array(map_size)/np.array(map_res))
        self.map_dig_center = (self.map_shape[0]//2,self.map_shape[1]//2)
        self.blur_kernel = blur_kernel
    def likelihood_map(self):
        Lmap = np.zeros_like(self.map)
        Lmap[self.map == 1] = 1
        #Lmap = self.map.copy()
        Lmap = cv2.GaussianBlur(Lmap,self.blur_kernel,0)
        return Lmap


