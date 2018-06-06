#!/usr/bin/python2.7

import rospy
import time
import math
import os
import numpy as np
import matplotlib.pyplot as plt

import sumo.vehicleControl as vehicleControl
from common.msg import Pose
from common.msg import Trajectory
from common.msg import DynamicObstacle
from common.msg import ObstacleMap

CAR_WIDTH = 3
trajectory = Trajectory()
trajectory_ready = False

def callback_trajectory(msg):
    global trajectory
    trajectory = msg
    global trajectory_ready
    trajectory_ready = True
    #print "callback traj:", trajectory

def init_sim():
    vehicleControl.init()
    vehicleControl.init_vehicle()

if __name__=="__main__":
    init_sim()

    rospy.init_node("sumo")
    rospy.Subscriber("/planning/trajectory", Trajectory, callback_trajectory)
    pub_localize = rospy.Publisher("/simulation/localize", Pose, queue_size=10)
    pub_obstacle = rospy.Publisher("/simulation/obstacles", ObstacleMap, queue_size = 10)
    rate = rospy.Rate(10)

    path = os.getcwd() + '/src/planning/log/vehicle_log'
    file_path = open(path, 'w')
    trajectory_log = open(os.getcwd() + '/src/planning/log/trajectory_log','w')

    while not rospy.is_shutdown():
        print "\n\n-------- one step ------------"
        localize = vehicleControl.get_localize()
        obs_map = vehicleControl.get_obstacles()
        pub_localize.publish(localize)
        pub_obstacle.publish(obs_map)
        print "vehicle state: t:", localize.timestamp, "x:", localize.x, "y:", localize.y, "v:", localize.velocity
        vehicleControl.do_step(trajectory, trajectory_ready)
        log_path = "%f\t%f\t%f\t%f\t%f\n" %(localize.timestamp,localize.x, localize.y, localize.velocity,localize.acceleration)
        file_path.write(log_path)
        rate.sleep()
        trajectory_log.write("trajectory\n")
        for pose in trajectory.poses:
            log_path = "%f\t%f\n" %(pose.timestamp, pose.velocity)
            #print pose.velocity
            trajectory_log.write(log_path)

    vehicleControl.destroy()
    file_path.close()
    trajectory_log.close()
