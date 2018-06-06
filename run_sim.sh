#!/bin/zsh
source devel/setup.zsh

#sumo-gui -c src/simulation/data/crossing.sumocfg --remote-port 1339
#echo "sumo done"

python2 src/simulation/python/runSim.py -g
