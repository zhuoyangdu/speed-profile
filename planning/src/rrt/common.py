#!/usr/bin/env python
'''
    Some common functions and define of geometry path class.
'''

import rospy
import math
from node import Node
from scipy import interpolate

def compute_distance(x1,x2):
    return math.sqrt(math.pow(x1[0]-x2[0],2) + math.pow(x1[1]-x2[1],2))

def estimate_velocity(parent_node, child_node):
    velocity = (parent_node.distance - child_node.distance)/(parent_node.time - child_node.time)
    return velocity

class GeometryPath(object):
    def __init__(self):
        # Path spline.
        x = []
        y = []
        file = open("../../data/RoadXY.txt")
        for line in file.readlines():
            sl = line.split("\t")
            x.append(float(sl[0]))
            y.append(float(sl[1]))
            #angle.append(float(sl[2]))
        length = [0]
        len_t = 0
        for i in range(1,len(x)):
            len_t = len_t + math.sqrt(math.pow(x[i]-x[i-1], 2) + math.pow(y[i]-y[i-1],2))
            length.append(len_t)
        self.path_x = x
        self.path_y = y
        self.spline_x = interpolate.splrep(length, x)
        self.spline_y = interpolate.splrep(length, y)
    '''
    def path_spline(self, length):
        sx = interpolate.splev(length, self.spline_x, der=0)
        sy = interpolate.splev(length, self.spline_y, der=0)
        return sx, sy
    '''
    def get_nearest_point(self,x,y):
        dis = []
        for i in range(0,len(self.path_x)):
            distance = math.sqrt(math.pow(x-path_x[i],2) + math.pow(y-path_y[i],2))
            dis.append(distance)
        index = dis.index(min(dis))
        return index

    def get_path_length(self,x,y):
        nearest_index = self.get_nearest_point(x,y)
        path_length = 0
        for i in range(0,index):
            path_length += compute_distance([path_x[i], path_y[i]],[path_x[i+1], path_y[i+1]])
        return path_length

    def path_spline(self, x, y, length):
        path_length = length + self.get_path_length(x,y)
        sx = interpolate.splev(path_length, self.spline_x, der=0)
        sy = interpolate.splev(path_length, self.spline_y, der=0)
        return sx,sy

geometry_path = GeometryPath()
