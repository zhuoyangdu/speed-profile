#!/usr/bin/env python
import matplotlib.pyplot as plt
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
            distance = math.sqrt(math.pow(x-self.path_x[i],2) + math.pow(y-self.path_y[i],2))
            dis.append(distance)
        index = dis.index(min(dis))
        return index

    def get_path_length(self,x,y):
        nearest_index = self.get_nearest_point(x,y)
        path_length = 0
        for i in range(0,nearest_index):
            path_length += math.sqrt(math.pow(self.path_x[i]-self.path_x[i+1],2) + math.pow(self.path_y[i]-self.path_y[i+1],2))
        return path_length

    def path_spline(self, path_length):
        sx = interpolate.splev(path_length, self.spline_x, der=0)
        sy = interpolate.splev(path_length, self.spline_y, der=0)
        return sx,sy

geometry_path = GeometryPath()
DT = 0.05
DS = 0.2
T_MAX = 5
V_MAX = 12
S_MAX = T_MAX * V_MAX
DANGER_DISTANCE = 10

collision_t = []
collision_s = []

tree_t = []
tree_s = []
tree_v = []
plt.figure("tree")

def plot_configuration_space():
    obs_x = 0
    obs_y = 0
    obs_theta = 0
    obs_vel = 0
    veh_s = 0

    file = open("../log/log_tree_17_07_22_15_29_43","r")
    lines = file.readlines(100000000000)
    for  line in lines:
        sl = line.split("\t")
        #print "sl", sl
        if sl[0] == "obstacle":
            obs_timestamp = float(sl[1])
            obs_x = float(sl[3])
            obs_y = float(sl[4])
            obs_theta = float(sl[5])
            obs_vel = float(sl[6])
        if sl[0] == "vehicle_state":
            veh_timestamp = float(sl[1])
            veh_x = float(sl[2])
            veh_y = float(sl[3])
            veh_theta = float(sl[4])
            veh_vel = float(sl[5])
            path_length = geometry_path.get_path_length(veh_x, veh_y)
            veh_s = path_length
            print veh_s, veh_x, veh_y
        if sl[0] == "steer":
            new_t = float(sl[2])
            new_s = float(sl[3])
            new_v = float(sl[4])
            parent_t = float(sl[6])
            parent_s = float(sl[7])
            parent_v = float(sl[8])
            plt.plot([new_t,parent_t],[new_s,parent_s],"r")
        if sl[0] == "path":
            path_size = int(sl[1])
            pt = []
            ps = []
            pv = []
            for i in range(0, path_size):
                pt.append(float(sl[3*i+2]))
                ps.append(float(sl[3*i+3]))
                pv.append(float(sl[3*i+4]))
            plt.plot(pt, ps, "g")
    #plt.show()

    #plt.figure('s-t space')
    for i in range(0, int(T_MAX/DT), 1):
        t = i * DT;
        for j in range(0, int(S_MAX/DS), 1):
            s = j * DS + veh_s
            ox = obs_x + obs_vel * t * math.sin(obs_theta)
            oy = obs_y + obs_vel * t * math.cos(obs_theta)
            [vx, vy] = geometry_path.path_spline(s)
            dis = math.sqrt(math.pow(ox-vx,2) + math.pow(oy-vy,2))
            if dis < DANGER_DISTANCE:
                collision_s.append(s)
                collision_t.append(t)
    plt.plot(collision_t,collision_s,".")
    plt.xlim(0,5)
    plt.ylim(veh_s, veh_s+S_MAX)
    plt.plot([0,T_MAX],[veh_s, veh_s+S_MAX],'--')
    plt.show()

plot_configuration_space()
