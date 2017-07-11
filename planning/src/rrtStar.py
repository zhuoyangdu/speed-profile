#!/usr/bin/env python
import rospy
import math
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

class GeometryPath(object):
    def __init__(self):
        # Path spline.
        x = []
        y = []
        file = open("../data/RoadXY.txt")
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
        self.spline_x = interpolate.splrep(length, x)
        self.spline_y = interpolate.splrep(length, y)

    def path_spline(self, length):
        sx = interpolate.splev(length, self.spline_x, der=0)
        sy = interpolate.splev(length, self.spline_y, der=0)
        return sx, sy

class  Node(object):
    """docstring for  Node"""
    def __init__(self, t,s):
        self.t = t
        self.s = s
        self.v = None
        self.parent_node = None

    def get_parent_node(self):
        return self.parent_node


def generate_trajectory(vehicle_state, obstacle_map):
    trajectory = Trajectory()
    rospy.loginfo("generate trajectory!")
    return trajectory

if __name__=="__main__":