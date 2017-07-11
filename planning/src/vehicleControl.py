import os
import sys
import subprocess
import time

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
    traci.start([sumoBinary, "-c", "../data/crossing.sumocfg"])
    print "Traci initialized."

def destroy():
    traci.close()
    print "Traci closed."

def init_vehicle(init_pos, init_speed, init_route):
    traci.vehicle.addFull(self_veh.get_id(), init_route,departPos=init_pos,departSpeed=init_speed)
    traci.vehicle.setSpeed(self_veh.get_id(), 0);
    print "Self vehicle initialized."

def get_localize():
    localize = Pose()
    [localize.x, localize.y] = traci.vehicle.getPosition(self_veh.get_id())
    localize.theta = traci.vehicle.getAngle(self_veh.get_id())
    localize.velocity = traci.vehicle.getSpeed(self_veh.get_id())
    localize.timestamp = traci.simulation.getCurrentTime()
    return localize

def do_step():
    # print traci.simulation.getCurrentTime()
    [x,y] = traci.vehicle.getPosition("veh5")
    ang = traci.vehicle.getAngle("veh5")
    print "route02", x,y,ang
    traci.simulationStep()

def get_obstacles():
    timestamp = traci.simulation.getCurrentTime()
    obs_map = ObstacleMap()
    for veh in traci.vehicle.getIDList():
        if veh != self_veh.get_id():
            obs_veh = DynamicObstacle()
            obs_veh.timestamp = timestamp
            obs_veh.id = veh
            (obs_veh.x,obs_veh.y) = traci.vehicle.getPosition(veh)
            obs_veh.theta = traci.vehicle.getAngle(veh)
            obs_veh.velocity =  traci.vehicle.getSpeed(veh)
            obs_map.dynamic_obstacles.append(obs_veh)
    return obs_map
