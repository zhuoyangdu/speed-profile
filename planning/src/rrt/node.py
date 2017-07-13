#!/usr/bin/env python
import rospy

class Node(object):
    def __init__(self, time, distance, self_id=-1):
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

    def print_node(self):
        print "node t:", self.time, " s:", self.distance," v:",self.velocity," id:",self.self_id," par_id:",self.parent_id