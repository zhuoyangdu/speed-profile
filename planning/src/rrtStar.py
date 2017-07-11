#!/usr/bin/env python
import rospy
import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt

from scipy import interpolate

from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap
from constants import *  #configuration file

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

class Node(object):
    def __init__(self, time, distance, self_id):
        self.time = time
        self.distance = distance
        self.self_id = self_id
        self.velocity = None
        self.cost = None
        self.parent_id = None

    def set_parent_node(self, parent_id):
        self.set_parent_id = parent_id

    def set_cost(self, cost):
        self.cost = cost

class Obstacles(object):
    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map

    def collision_free(self, node1, node2):
        return

class Tree(object):
    def __init__(self, t0, s0, v0):
        first_node = Node(t0, s0, 0);
        first_node.velocity = v0
        first_node.cost = 0
        self.nodes = [first_node]

    def add_node(self, node):
        self.nodes.append(node)

    def nearest(self, node):
        dist = []
        for tmp_node in self.nodes:
            delta_s = tmp_node.distance - node.distance
            delta_t = tmp_node.time - node.time
            if delta_s<=0 or delta_t <=0:
                dist.append(float('Inf'))
            else:
                vel = delta_s / delta_t
                acc = (tmp_node.velocity - vel) / delta_t
                if vel > MAX_VEL or acc > MAX_ACC:
                    dist.append(float('Inf'))
                else:
                    dist.append(math.sqrt(math.pow(delta_s/S_MAX,2)+math.pow(delta_t/T_MAX,2)))
        nearest_index = dist.index(min(dist))
        print "nearest node:", nearest_index
        print "min_distance:", min(dist)
        return self.nodes[nearest_index]

    def steer(self, nearest_node, sample):
        k = self.estimate_velocity(near_node - sample)
        new_time = near_node.time + DT
        new_distance = near_node.distance + DT * k
        new_id = self.get_tree_size()
        new_node = Node(new_time, new_distance, new_id)
        return new_node

    def node_cost(self, parent_node, child_node):
        return

    def near_lower_region(self, node):
        min_time = node.time + LOWER_RANGE[0]
        min_distance = node.distance + LOWER_RANGE[1]
        near_region = []
        for tmp_node in self.nodes:
            if tmp_node.time < node.time and tmp_node.time > min_time and tmp_node.distance < node.distance and tmp_node.distance > min_distance:
                near_region.append(tmp_node)
        return near_region

    def near_upper_region(self, node):
        max_time = node.time + UPPER_RANGE[0]
        max_distance = node.distance + UPPER_RANGE[1]
        near_region = []
        for tmp_node in self.nodes:
            if tmp_node.time > node.time and tmp_node.time < max_time and tmp_node.distance > node.distance and tmp_node.distance < max_distance:
                near_region.append(tmp_node)
        return near_region

    def vertex_feasible(self, parent_node, child_node):
        # Kinematic feasible
        kinematic_feasible = True
        if parent_node.time <= child_node.time or parent_node.distance < child_node.distance:
            print "error in find nodes."
            return False
        vel = self.estimate_velocity(parent_node, child_node)
        if abs(vel) > MAC_VEL:
            return False
        acc = self.estimate_acceleration(parent_node, child_node)
        if abs(acc) > MAX_ACC:
            return False

        collision_free = Obstacles.collision_free()
        if collision_free:
            return True
        else:
            return False

    def weighting_cost(self, cost):
        return

    def get_tree_size(self):
        return len(self.nodes)

    def estimate_velocity(self, parent_node, child_node):
        velocity = (parent_node.distance - child_node.distance)/(parent_node.time - child_node.time)
        return velocity

    def estimate_acceleration(self, parent_node, child_node):
        vel = estimate_velocity(parent_node, child_node)
        parent_vel = parent_node.velocity
        acc = (parent_vel - vel) / (parent_node.time - child_node.time)
        return acc

    def rebuild_tree(self, near_node, new_node):
        return

    def extend(self, sample):
        node_valid = False
        nearest_node = self.nearest(sample)
        new_node = self.steer(nearest_node, sample)
        if self.vertex_feasible(nearest_node, new_node):
            node_valid = True
            min_node = nearest_node
            cost_min = self.node_cost(min_node, new_node)
            near_region = self.near_lower_region(new_node)
            for near_node in near_region:
                if self.vertex_feasible(near_node, new_node):
                    cost_near = self.node_cost(near_node, new_node)
                    if self.weighting_cost(cost_near) < self.weighting_cost(cost_min):
                        cost_min = cost_near
                        min_node = near_node
            new_node.parent_id = min_node.self_id
            new_node.self_id = self.get_tree_size()
            new_node.velocity = self.estimate_velocity(min_node, new_node)
            self.add_node(new_node)

            near_region = self.near_upper_region(new_node)
            for near_node in near_region:
                if self.vertex_feasible(near_node, new_node):
                    cost_near = near_node.cost
                    cost_new = self.node_cost(new_node, near_node)
                    if self.weighting_cost(cost_near) > self.weighting_cost(cost_new):
                        self.rebuild_tree(near_node, new_node)

    def get_parent_path(self, node):
        path = [node]
        while node.parent_id != -1:
            parent_node = self.nodes[node.parent_id]
            path.insert(0, parent_node)
            node = parent_node
        return path

    def paint_tree():
        return

def reaching_goal(node):
    return

def get_path_cost(path):
    return

def random_sample():
    return Node(random(0,T_MAX), random(0, S_MAX))

def generate_trajectory(vehicle_state, obstacle_map):
    trajectory = Trajectory()
    rospy.loginfo("generate trajectory!")

    tree = Tree(INIT_TIME, INIT_DIST, INIT_VEL)
    cost = Cost()
    i = 0
    N_feasible = 0
    N_path = 0
    min_cost = Inf
    while i < MAX_FAILED_ATTEMPTS:
        i = i + 1
        sample = random_sample()
        [node_valid, new_node] = tree.extend(sample)
        if node_valid:
            N_feasible = N_feasible + 1
            if reaching_goal(new_node):
                path = tree.get_parent_path(new_node)
                path_cost = get_path_cost(path_cost)
                if path_cost < min_cost:
                    min_cost = path_cost
                    path_min_cost = path
                N_path = N_path + 1
                print "Found", N_path, "paths."
                if N_path > 10:
                    break
    return trajectory

if __name__=="__main__":

