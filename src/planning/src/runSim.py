#!/usr/bin/python2.7

import rospy
import time
import math
import os
import numpy as np
import matplotlib.pyplot as plt

import sumo.vehicleControl as vehicleControl
from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

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

def plot_car(vehicle):
    theta = vehicle.theta/180*math.pi
    x1 = vehicle.x + CAR_WIDTH*math.sin(theta)
    y1 = vehicle.y + CAR_WIDTH*math.cos(theta)
    x2 = vehicle.x - CAR_WIDTH*math.sin(theta)
    y2 = vehicle.y - CAR_WIDTH*math.cos(theta)
    x3 = x2 + CAR_WIDTH/2*math.cos(theta)
    y3 = y2 - CAR_WIDTH/2*math.sin(theta)
    x4 = x2 - CAR_WIDTH/2*math.cos(theta)
    y4 = y2 + CAR_WIDTH/2*math.sin(theta)
    x = [x1,x3,x4,x1]
    y = [y1,y3,y4,y1]
    return x,y

def print_map(localize, obs_map):
    plt.figure("map")
    plt.clf()
    plt.plot([495,495],[0,495],"b",lw=0.5)
    plt.plot([495,495],[505,1000],"b",lw=0.5)
    plt.plot([505,505],[0,495],"b",lw=0.5)
    plt.plot([505,505],[505,1000],"b",lw=0.5)
    plt.plot([0,495],[495,495],"b",lw=0.5)
    plt.plot([505,1000],[495,495],"b",lw=0.5)
    plt.plot([0,495],[505,505],"b",lw=0.5)
    plt.plot([505,1000],[505,505],"b",lw=0.5)
    plt.plot([500,500],[0,1000],"y--",lw=0.5)
    plt.plot([0,1000],[500,500],"y--",lw=0.5)
    plt.xlim([400,600])
    plt.ylim([400,600])
    #print "self:", localize.theta
    [x,y] = plot_car(localize)
    plt.plot(x,y,"r")
    # plt.plot(localize.x, localize.y,'ro')
    for obs in obs_map.dynamic_obstacles:
        [x,y] = plot_car(obs)
        plt.plot(x,y,"b")
        #print obs.id, obs.x, obs.y, obs.theta
    plt.pause(0.001)
    plt.draw()
    return

if __name__=="__main__":
    init_sim()

    rospy.init_node("sumo")
    rospy.Subscriber("/planning/trajectory", Trajectory, callback_trajectory)
    pub_localize = rospy.Publisher("/simulation/localize", Pose, queue_size=10)
    pub_obstacle = rospy.Publisher("/simulation/obstacles", ObstacleMap, queue_size = 10)
    rate = rospy.Rate(10)

    path = os.getcwd() + '/planning/log/vehicle_log'
    file_path = open(path, 'w')
    trajectory_log = open(os.getcwd() + '/planning/log/trajectory_log','w')

    while not rospy.is_shutdown():
        localize = vehicleControl.get_localize()
        obs_map = vehicleControl.get_obstacles()
        # print_map(localize, obs_map)
        #print localize
        #print obs_map
        pub_localize.publish(localize)
        pub_obstacle.publish(obs_map)
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
