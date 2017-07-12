#!/usr/bin/env python
import rospy
import math
from node import Node
from geometryPath import GeometryPath

geometry_path = GeometryPath()

def compute_distance(x1,x2):
    return math.sqrt(math.pow(x1[0]-x2[0],2) + math.pow(x1[1]-x2[1],2))

def estimate_velocity(parent_node, child_node):
    velocity = (parent_node.distance - child_node.distance)/(parent_node.time - child_node.time)
    return velocity
