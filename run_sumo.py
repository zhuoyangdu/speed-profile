#!/usr/bin/env python
"""
@file    run_sumo.py
@author  Zhuoyang Du
@data    2017-07-06
"""

import os
import sys
import optparse
import subprocess

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

# this is the main entry point of this script
if __name__=="__main__":
    sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/crossing.sumocfg"])
    step = 0
    while step < 1000:
        traci.simulationStep()
        step+=1
    traci.close()
