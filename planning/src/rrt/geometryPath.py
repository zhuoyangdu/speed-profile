#!/usr/bin/env python
import rospy
import math

from scipy import interpolate

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