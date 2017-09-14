import os
import sys
import subprocess
import time
import math
from constants import *

from planning.msg import Pose
from planning.msg import Trajectory
from planning.msg import DynamicObstacle
from planning.msg import ObstacleMap

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
    file_path = os.getcwd() + '/planning/data/crossing.sumocfg'
    try:
        traci.start([sumoBinary, "-c", file_path])
    except Exception as e:
        print "Error finding simulation files."
    print "Traci initialized."

def destroy():
    traci.close()
    print "Traci closed."

def init_vehicle():
    traci.vehicle.addFull(self_veh.get_id(), INIT_ROUTE,departPos=INIT_POS,departSpeed=INIT_SPEED)
    traci.vehicle.setSpeedMode(self_veh.get_id(), 0)
    traci.vehicle.setSpeed(self_veh.get_id(), float(INIT_SPEED))

    traci.simulationStep()

    for veh in traci.vehicle.getIDList():
        if veh!= self_veh.get_id():
            traci.vehicle.setSpeedMode(veh, 0)
            traci.vehicle.setSpeed(veh, 5)
            print "id:", veh
            print "pos:", traci.vehicle.getPosition(veh)
            print "angle:", traci.vehicle.getAngle(veh)
            print "vel:", traci.vehicle.getSpeed(veh)
    print "Self vehicle initialized."

    #print traci.vehicle.getShapeClass(self_veh.get_id())

def get_localize():
    localize = Pose()
    [localize.x, localize.y] = traci.vehicle.getPosition(self_veh.get_id())
    localize.theta = traci.vehicle.getAngle(self_veh.get_id()) / 180 * math.pi
    localize.velocity = traci.vehicle.getSpeed(self_veh.get_id())
    localize.timestamp = float(traci.simulation.getCurrentTime()/1000.0)
    localize.length = traci.vehicle.getDistance(self_veh.get_id())
    localize.acceleration = traci.vehicle.getAccel(self_veh.get_id())
    return localize

def do_step(trajectory, trajectory_ready):
    #print traci.simulation.getCurrentTime()
    tc = float(traci.simulation.getCurrentTime()/1000.0)

    if trajectory_ready:
        index = 0
        for k in range(0, len(trajectory.poses)):
            if tc < trajectory.poses[k].timestamp:
                index = k
                break
        vel = trajectory.poses[index].velocity

        print "time", traci.simulation.getCurrentTime(),"tc:", tc, "ref_vel:", vel, "current velocity:", traci.vehicle.getSpeed(self_veh.get_id())
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
