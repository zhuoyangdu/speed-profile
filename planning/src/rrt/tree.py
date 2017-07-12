#!/usr/bin/env python
import rospy
import math
from node import Node
from common import *

class Tree(object):
    def __init__(self, t0, s0, v0):
        first_node = Node(t0, s0, 0);
        first_node.velocity = v0
        first_node.cost = 0
        self.nodes = [first_node]

    def add_node(self, node):
    self.nodes.append(node)
