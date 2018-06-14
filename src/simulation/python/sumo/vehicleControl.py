#!/usr/bin/python2.7
import os
import sys
import subprocess
import time
import math
from constants import *

from common.msg import Pose
from common.msg import Trajectory
from common.msg import DynamicObstacle
from common.msg import ObstacleMap

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")
import traci
import traci.constants as tc
from optparse import OptionParser

class Vehicle(object):
    veh_id = None
    def __init__(self, veh_id):
        self.veh_id = veh_id

    def get_id(self):
        return self.veh_id

self_veh = Vehicle("self_veh")

def init():
    optParser = OptionParser()
    optParser.add_option("-g", "--gui", action="store_true", dest="gui",default=False, help="run with GUI")
    (options, args) = optParser.parse_args()
    sumoExe = 'sumo'
    if options.gui:
        sumoExe = 'sumo-gui'
    sumoBinary = checkBinary(sumoExe)
    file_path = os.getcwd() + '/src/simulation/data/complex.sumocfg'
    try:
        traci.start([sumoBinary, "-c", file_path])
    except Exception as e:
        print "Error finding simulation files."
    print "Traci initialized."

def destroy():
    traci.close()
    print "Traci closed."

def init_vehicle():

    #traci.vehicle.addFull("self_veh", "route04", departPos="100", departLane="2", arrivalLane="2")
    traci.vehicle.addFull("self_veh", "route04", departLane="2", departPos="100", typeID="self")
    traci.vehicle.setSpeedMode(self_veh.get_id(), 0)
    traci.vehicle.setSpeed(self_veh.get_id(), float(INIT_SPEED))

    #traci.vehicle.moveToXY(self_veh.get_id(), "L10", 1, INIT_X, INIT_Y, 90)

    traci.simulationStep()

    for veh in traci.vehicle.getIDList():
        if veh!= self_veh.get_id():
            traci.vehicle.setSpeedMode(veh, 0)
            traci.vehicle.setSpeed(veh, 5)
            #print "id:", veh
            #print "pos:", traci.vehicle.getPosition(veh)
            #print "angle:", traci.vehicle.getAngle(veh)
            #print "vel:", traci.vehicle.getSpeed(veh)
    print "type:", traci.vehicle.getVehicleClass("self_veh")
    print "type id:", traci.vehicle.getTypeID("self_veh")
    print "init position: ", traci.vehicle.getPosition("self_veh")
    print "route: ", traci.vehicle.getRoute("self_veh")
    print "distance: ", traci.vehicle.getDistance("self_veh")
    print "LaneID:", traci.vehicle.getLaneID("self_veh")
    print "Self vehicle initialized."

    #print traci.vehicle.getShapeClass(self_veh.get_id())
previous_time = 0
previous_velocity = 0

def get_localize():
    localize = Pose()
    [localize.x, localize.y] = traci.vehicle.getPosition(self_veh.get_id())
    localize.theta = traci.vehicle.getAngle(self_veh.get_id()) / 180 * math.pi
    localize.velocity = traci.vehicle.getSpeed(self_veh.get_id())
    localize.timestamp = float(traci.simulation.getCurrentTime()/1000.0)
    localize.length = traci.vehicle.getDistance(self_veh.get_id())
    #localize.acceleration = traci.vehicle.getAccel(self_veh.get_id())
    global previous_velocity, previous_time
    if previous_time==0:
        localize.acceleration = 0
    else:
        localize.acceleration = (previous_velocity- localize.velocity)/(previous_time-localize.timestamp)
        # print "acceleration :", localize.acceleration
    previous_velocity = localize.velocity
    previous_time = localize.timestamp
    return localize

def do_step(trajectory, trajectory_ready):
    #print traci.simulation.getCurrentTime()
    print "Trajectory:"
    for pose in trajectory.poses:
        print "t:", pose.timestamp, "x:", pose.x, "y:", pose.y, "v:", pose.velocity

    tc = float(traci.simulation.getCurrentTime()/1000.0)

    print "tc:", tc

    if trajectory_ready:
        index1 = 0
        for k in range(0, len(trajectory.poses)):
            if  trajectory.poses[k].timestamp > tc:
                index1 = k
                break
        index2 = k-1
        t1 = trajectory.poses[index1].timestamp
        t2 = trajectory.poses[index2].timestamp
        v1 = trajectory.poses[index1].velocity
        v2 = trajectory.poses[index2].velocity
        x1 = trajectory.poses[index1].x
        x2 = trajectory.poses[index2].x
        y1 = trajectory.poses[index1].y
        y2 = trajectory.poses[index2].y
        
        theta1 = trajectory.poses[index1].theta
        theta2 = trajectory.poses[index2].theta

        x = (x1 - x2) * (tc - t2)/(t1 - t2) + x1
        y = (y1 - y2) * (tc - t2)/(t1 - t2) + y1
        theta = (theta1 - theta2) * (tc - t2)/(t1 - t2) + theta1

        vel = (v1-v2)*(tc-t2)/(t1-t2) + v2
        a = (v1-v2)/(t2-t1)
        s = v1 * (tc - t1) + 0.5 * a * (tc - t1) * (tc - t1)
        s0 = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
        if (s0 == 0) :
            k = 1
        else:
            k = s / s0
        xt = (x2 - x1) * k + x1
        yt = (y2 - y1) * k + y1

        #if (x < 0):
        #    traci.vehicle.moveToXY(self_veh.get_id(),"L10",1, xt, yt, theta/3.1415*180)
        #else:
        #    traci.vehicle.moveToXY(self_veh.get_id(), "L03", 1, xt, yt, theta/3.1415*180)

        traci.vehicle.setSpeed(self_veh.get_id(), vel)
        
    traci.simulationStep()

def get_obstacles():
    (vehicle_x, vehicle_y) = traci.vehicle.getPosition(self_veh.get_id())
    timestamp = float(traci.simulation.getCurrentTime()/1000.0)
    obs_map = ObstacleMap()
    for veh in traci.vehicle.getIDList():
        if veh != self_veh.get_id():
            obs_veh = DynamicObstacle()
            obs_veh.timestamp = timestamp
            obs_veh.id = veh
            (obs_veh.x,obs_veh.y) = traci.vehicle.getPosition(veh)
            obs_veh.theta = traci.vehicle.getAngle(veh) / 180 * math.pi
            obs_veh.velocity =  traci.vehicle.getSpeed(veh)
            dis = math.sqrt(math.pow(obs_veh.x-vehicle_x,2)+math.pow(obs_veh.y-vehicle_y,2))
            if dis < OBSTACLE_RANGE:
                obs_map.dynamic_obstacles.append(obs_veh)
    return obs_map
