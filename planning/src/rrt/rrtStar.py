#!/usr/bin/env python
import rospy
import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt

from node import Node
from geometryPath import GeometryPath
from constants import *  #configuration file
from common import *
from path import *

from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

obstacles = []

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

    def get_smooth_cost(self, parent_node, child_node):
        parent_vel = parent_node.velocity
        child_vel = estimate_velocity(parent_node, child_node)
        smoothness = abs(parent_vel - child_vel)
        return smoothness

    def get_vel_error(self,parent_node,child_node):
        velocity = estimate_velocity(parent_node, child_node)
        vel_error = abs(velocity - V_GOAL)
        return vel_error

    def node_cost(self, parent_node, child_node):
        path = self.get_parent_path(parent_node)
        path.append(child_node)
        risk = Obstacles.risk_assessment(path)
        smoothness = self.get_smooth_cost(parent_node, child_node)
        e_vel = self.get_vel_error(parent_node, child_node)
        cost = [risk, smoothness, e_vel]
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
        return KR * cost[0] + KS * cost[1] + KV * cost[2]

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
        near_node.parent_id = new_node.self_id
        near_node.velocity = self.estimate_velocity(new_node, near_node)
        near_node.cost = self.node_cost(new_node, near_node)
        self.nodes[near_node.self_id] = near_node
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

    def print_tree():
        for node in self.nodes:
            rospy.loginfo("t: %d, s:%.3f. v: %.3f, id: %d, par_id:%d.", node.time, node.distance, node.velocity, node.self_id, node.parent_id)

class Planning(object):
    def __init__(self, vehicle_state, obstacle_map):
        obstacles = Obstacles(obstacle_map)
        self.vehicle_state = vehicle_state
        print "init tree:", "time:", vehicle_state.timestamp, vehicle_state.length, vehicle_state.velocity
        self.tree = Tree(self.vehicle_state.timestamp, self.vehicle_state.length, self.vehicle_state.velocity)

    def generate_trajectory(self):
        i = 0
        N_feasible = 0
        N_path = 0
        min_cost = float("Inf")
        while i < MAX_FAILED_ATTEMPTS:
            i = i + 1
            sample = self.random_sample()
            [node_valid, new_node] = self.tree.extend(sample)
            if node_valid:
                N_feasible = N_feasible + 1
                if self.reaching_goal(new_node):
                    path = self.tree.get_parent_path(new_node)
                    path_cost = get_path_cost(obstacles, path_cost)
                    if path_cost < min_cost:
                        min_cost = path_cost
                        path_min_cost = path
                    N_path = N_path + 1
                    print "Found", N_path, "paths."
                    if N_path > 10:
                        break
        return

    def random_sample(self):
        return Node(random(0,T_MAX), random(0, S_MAX))

    def reaching_goal(self, node):
        if abs(node.time - T_GOAL) < 0.3:
            return True
        return False

