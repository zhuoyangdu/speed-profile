#!/usr/bin/env python
import rospy
import math
from constants import *
from obstacles import Obstacles
from node import Node

def print_path(path):
    for node in path:
        node.print_node()

def get_path_cost(obstacles, path):
    risk = obstacles.risk_assessment(path)
    smoothness = path_smoothness(path)
    e_vel = path_vel_error(path)
    return risk, smoothness, e_vel

def path_smoothness(path):
    sum_acc = 0
    for i in range(1,len(path)-1):
        node = path[i]
        parent_node = path[i-1]
        delta_v = node.velocity - parent_node.velocity
        delta_t = node.time - parent_node.time
        acc = delta_v/delta_t
        sum_acc = sum_acc + acc
    return sum_acc

def path_vel_error(path):
    ev = 0
    for node in path:
        ev = ev + abs(node.velocity - V_GOAL)
    return ev