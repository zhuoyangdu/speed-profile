#!/usr/bin/env python
import rospy
import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt

from node import Node
from constants import *  #configuration file
from common import *
from path import *
from visualization import print_map

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
        first_node.self_id = 0
        first_node.parent_id = -1
        self.nodes = [first_node]
        # first_node.print_node()

    def add_node(self, node):
        self.nodes.append(node)

    def nearest(self, node):
        dist = []
        for tmp_node in self.nodes:
            delta_s = node.distance - tmp_node.distance
            delta_t = node.time - tmp_node.time
            #print "delta_s:", delta_s," delta_t:", delta_t
            if delta_s<=0 or delta_t <=0:
                dist.append(float("Inf"))
            else:
                vel = delta_s / delta_t
                acc = (tmp_node.velocity - vel) / delta_t
                if vel > MAX_VEL or acc > MAX_ACC:
                    # print "velocity or acceleration is too large:", vel, acc
                    dist.append(float("Inf"))
                else:
                    dist.append(math.sqrt(math.pow(delta_s/S_MAX,2)+math.pow(delta_t/T_MAX,2)))
        nearest_index = dist.index(min(dist))

        min_dist = min(dist)
        if min_dist == float("Inf"):
            #print "[invalid sample] There isn't any node meeting the requirement."
            return None
        else:
            #print "[nearest] nearest node: "
            #self.nodes[nearest_index].print_node()
            # print "The distance of the nearest node is:", min(dist)
            return self.nodes[nearest_index]

    def steer(self, near_node, sample):
        k = estimate_velocity(near_node, sample)
        new_time = near_node.time + DT
        new_distance = near_node.distance + DT * k
        new_id = self.get_tree_size()
        new_node = Node(new_time, new_distance, new_id, near_node.self_id)
        #print "[steer] new node:"
        #new_node.print_node()
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
        #print "[node cost] parent path:"
        #print_path(path)

        # TODO: the range of the three metrics should be 0-1
        risk = obstacles.risk_assessment(path)
        #print "[node cost] risk assessment:", risk
        smoothness = self.get_smooth_cost(parent_node, child_node)
        #print "[node cost] smoothness:", smoothness
        e_vel = self.get_vel_error(parent_node, child_node)
        #print "[node cost] e_vel:", e_vel
        return [risk, smoothness, e_vel]

    def near_lower_region(self, node):
        min_time = node.time - LOWER_RANGE[0]
        min_distance = node.distance - LOWER_RANGE[1]
        near_region = []
        for tmp_node in self.nodes:
            if tmp_node.time < node.time and tmp_node.time > min_time and tmp_node.distance < node.distance and tmp_node.distance > min_distance:
                near_region.append(tmp_node)
        return near_region

    def near_upper_region(self, node):
        max_time = node.time + UPPER_RANGE[0]
        max_distance = node.distance + UPPER_RANGE[1]
        near_region = []
        #node.print_node()
        #print_path(self.nodes)
        # print "min_time:", node.time, " max_time:", max_time
        # print "min_distance:", node.distance, "max_dis:",max_distance 
        for tmp_node in self.nodes:
            if tmp_node.time > node.time and tmp_node.time < max_time and tmp_node.distance > node.distance and tmp_node.distance < max_distance:
                # print "near node:"
                # tmp_node.print_node()
                near_region.append(tmp_node)
        return near_region

    def vertex_feasible(self, parent_node, child_node):
        # Kinematic feasible
        kinematic_feasible = True
        if parent_node.time >= child_node.time or parent_node.distance > child_node.distance:
            # print "[feasible] error in find nodes."
            return False
        vel = estimate_velocity(parent_node, child_node)
        # print "[feasible] vel", vel
        if abs(vel) > MAX_VEL:
            return False
        acc = self.estimate_acceleration(parent_node, child_node)
        # print "[feasible] acc", acc
        if abs(acc) > MAX_ACC:
            return False

        # print "[feasible] kinematic feasible!"
        collision_free = obstacles.collision_free(parent_node, child_node)
        if collision_free:
            #print "[feasible] node feasible!"
            return True
        else:
            # print "[invalid vertex] not feasible!"
            return False

    def weighting_cost(self, cost):
        #print "[weighting cost]:" ,cost[0], cost[1], cost[2]
        return KR * cost[0] + KS * cost[1] + KV * cost[2]

    def get_tree_size(self):
        return len(self.nodes)

    def estimate_acceleration(self, parent_node, child_node):
        vel = estimate_velocity(parent_node, child_node)
        parent_vel = parent_node.velocity
        acc = (parent_vel - vel) / (parent_node.time - child_node.time)
        return acc

    def rebuild_tree(self, near_node, new_node):
        near_node.parent_id = new_node.self_id
        near_node.velocity = estimate_velocity(new_node, near_node)
        near_node.cost = self.node_cost(new_node, near_node)
        self.nodes[near_node.self_id] = near_node
        return

    def extend(self, sample):
        # Represents the validation of the sample
        node_valid = False

        # Gets the nearest node of the sample, if None, return.
        nearest_node = self.nearest(sample)
        if nearest_node is None:
            return node_valid, Node(-1,-1,-1)

        # Steer function.
        new_node = self.steer(nearest_node, sample)

        # Rewire tree
        if self.vertex_feasible(nearest_node, new_node):
            #print "tree:"
            #print_path(self.nodes)
            node_valid = True
            min_node = nearest_node
            cost_min = self.node_cost(min_node, new_node)
            #print "[node cost]: the cost of the nearest node:",cost_min

            near_region = self.near_lower_region(new_node)
            #print "[near region]:"
            #print_path(near_region)
            for near_node in near_region:
                if self.vertex_feasible(near_node, new_node):
                    cost_near = self.node_cost(near_node, new_node)
                    #print "[cost near]:", cost_near
                    if self.weighting_cost(cost_near) < self.weighting_cost(cost_min):
                        #print "[rewire]: change the parent node of the new node!"
                        cost_min = cost_near
                        min_node = near_node
            new_node.parent_id = min_node.self_id
            new_node.self_id = self.get_tree_size()
            new_node.velocity = estimate_velocity(min_node, new_node)
            new_node.cost = cost_min
            self.add_node(new_node)

            near_region = self.near_upper_region(new_node)
            for near_node in near_region:
                if self.vertex_feasible(new_node, near_node):
                    cost_near = near_node.cost
                    cost_new = self.node_cost(new_node, near_node)
                    #print "cost new:", cost_new, " cost near:", cost_near
                    if self.weighting_cost(cost_near) > self.weighting_cost(cost_new):
                        #print "[rewire]: rebuild tree!"
                        self.rebuild_tree(near_node, new_node)
            return node_valid, new_node
        else:
            #print "[invalid node] no feasible!"
            return node_valid, Node(-1,-1,-1)

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
        print "[tree]"
        for node in self.nodes:
            node.print_node()

class Planning(object):
    def __init__(self, vehicle_state, obstacle_map):
        vehicle_state.length = geometry_path.get_path_length(vehicle_state.x, vehicle_state.y)
        self.vehicle_state = vehicle_state
        self.v0 = vehicle_state.velocity

        vehicle_state.timestamp = 0
        self.tree = Tree(0, self.vehicle_state.length, self.v0)

        global obstacles
        obstacles = Obstacles(obstacle_map)
        for obs in obstacles.obstacles:
            obs.timestamp = obs.timestamp - vehicle_state.timestamp

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
                    print "[path]: found path:"
                    print_path(path)
                    [risk, smoothness, e_vel] = get_path_cost(obstacles, path)
                    path_cost = risk + smoothness + e_vel
                    print "[path cost]:", path_cost
                    if path_cost < min_cost:
                        min_cost = path_cost
                        path_min_cost = path
                    N_path = N_path + 1
                    print "Found", N_path, "paths."
                    if N_path > 10:
                        break
            #print ""
        return

    def random_sample(self):
        sample = Node(random.uniform(0,T_MAX), random.uniform(0,S_MAX)+self.vehicle_state.length)
        # print "[sample]", sample.time, sample.distance
        return sample

    def reaching_goal(self, node):
        if abs(node.time - T_GOAL) < 0.3:
            return True
        return False

if __name__=="__main__":
    localize = Pose()
    localize.timestamp = 54000.4
    localize.x = 502.55
    localize.y = 481.2
    localize.theta = 0
    localize.length = 0
    localize.velocity = 6

    obstacle_map = ObstacleMap()
    obs = DynamicObstacle()
    obs.timestamp = 54000.4
    obs.id = "veh2"
    obs.x = 519.9628
    obs.y = 502.55
    obs.theta = 270.0/180*math.pi
    obs.velocity = 8
    obstacle_map.dynamic_obstacles = [obs]

    # print_map(localize, obstacle_map)
    t0 = time.clock()
    planning = Planning(localize, obstacle_map)
    planning.generate_trajectory()
    print "time passed:", time.clock() - t0

