#!/usr/bin/env python
import rospy
import math
from node import Node
from common import *
from constants import *

from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

def nonlinear_risk(dist):
    if dist < DANGER_DISTANCE:
        print "error in feasible vertex"
    if dist > SAFE_DISTANCE:
        risk = 0
    else:
        risk = K_RISK / (dist - DANGER_DISTANCE)
    return risk

class Obstacles(object):
    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map
        self.obstacles = obstacle_map.dynamic_obstacles

    def collision_free(self, node1, node2):
        for obs in self.obstacle_map.dynamic_obstacles:
            obs_x = obs.x
            obs_y = obs.y
            obs_ang = obs.theta
            obs_vel = obs.velocity
            for t in range(int(node1.time*10), int(node2.time*10+1), 1):
                t = t/10.0
                # TODO: confirm coordinate
                obs_pos_x = obs_x + obs_vel * t * math.sin(obs_ang)
                obs_pos_y = obs_y + obs_vel * t * math.cos(obs_ang)

                vehicle_vel = estimate_velocity(node1, node2)
                s = node1.distance + vehicle_vel * (t - node1.time)
                [vehicle_pos_x,vehicle_pos_y] = geometry_path.path_spline(s)
                dis = compute_distance([vehicle_pos_x, vehicle_pos_y],[obs_pos_x, obs_pos_y])
                # print "[collision free] vehicle:", vehicle_pos_x, vehicle_pos_y
                # print "[collision free] obstacles:", obs_pos_x, obs_pos_y
                # print "[collision free] distance:", dis, "t:",t
                if dis <= DANGER_DISTANCE:
                    return False
        return True

    def risk_assessment(self, path):
        min_dis = []
        for obs in self.obstacle_map.dynamic_obstacles:
            obs_id = obs.id
            obs_x = obs.x
            obs_y = obs.y
            obs_ang = obs.theta 
            obs_vel = obs.velocity
            dis = []
            for node in path:
                s = node.distance
                [x,y] = geometry_path.path_spline(s)
                obs_pos_x = obs_x + obs_vel * node.time * math.sin(obs_ang)
                obs_pos_y = obs_y + obs_vel * node.time * math.cos(obs_ang)
                ss = compute_distance([x,y],[obs_pos_x, obs_pos_y])
                dis.append(ss)
            min_dis.append(min(dis))
        print "[DCE] min distance:", min_dis
        min_min_dis = min(min_dis)
        risk = nonlinear_risk(min_min_dis)
        return risk