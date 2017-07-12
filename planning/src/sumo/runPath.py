#!/usr/bin/python
# -*- coding: UTF-8 -*-

import os
import sys
import subprocess
import time

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

optParser = OptionParser()
optParser.add_option("-g", "--gui", action="store_true", dest="gui",default=False, help="run with GUI")
(options, args) = optParser.parse_args()
sumoExe = 'sumo'
if options.gui:
    sumoExe = 'sumo-gui'
sumoBinary = checkBinary(sumoExe)
traci.start([sumoBinary, "-c", "../data/crossing.sumocfg"])
print "Traci initialized."

path_file = open("../data/RoadXY.txt", "w")

step = 0
while step < 5000:
    try:
        traci.simulationStep()
        step = step + 1
        [x,y] = traci.vehicle.getPosition("veh5")
        ang = traci.vehicle.getAngle("veh5")
        length = traci.vehicle.getLength("veh5")
        shape = traci.vehicle.getShapeClass("veh5")
        dis = traci.vehicle.getDistance("veh5")
        log_path = "%f\t%f\t%f\t%f\n" %(x,y,ang,dis)
        path_file.write(log_path)
        print "route02:", x,y,ang,dis
        print "shape:", shape
    except Exception as e:
        print e
        break

traci.close()
path_file.close()