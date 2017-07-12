#!/usr/bin/env python
import rospy

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

    def print_node(self):
        rospy.loginfo("node: t %d, s %.3f, v %.3f, id %d, par_id %d", self.time, self.distance, self.velocity, self.self_id, self.parent_id)
