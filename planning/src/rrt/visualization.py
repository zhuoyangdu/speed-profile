#!/usr/bin/env python
import math
import matplotlib.pyplot as plt
from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

from constants import *

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
    print "self:", localize.theta
    [x,y] = plot_car(localize)
    plt.plot(x,y,"r")
    # plt.plot(localize.x, localize.y,'ro')
    for obs in obs_map.dynamic_obstacles:
        [x,y] = plot_car(obs)
        plt.plot(x,y,"b")
        print obs.id, obs.x, obs.y, obs.theta
    plt.pause(0.001)
    plt.draw()
    return
