#!/bin/zsh
source devel/setup.zsh

# python2 src/simulation/scripts/runSim.py -g

sumo-gui -c src/simulation/data/crossing.sumocfg --remote-port 1339
echo "sumo done"

