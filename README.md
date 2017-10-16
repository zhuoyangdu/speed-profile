# Speed profile

While the sumo config file changes, use the following command to generate the .net.xml file:
```
netconvert -c crossing.netcfg
```
To run planning node:
```
source devel/setup.bash
roslaunch planning planning_node.launch
```

Before you run the simulation node, you should set the environment varaible SOMO_HOME first, like:
```
export SUMO_HOME=/home/robot/Documents/sumo-0.28.0/
```

To run simulation node:
```
source devel/setup.bash
python planning/src/runSim.py
```
or add -g after runSim.py to open the visualization of sumo.

Or simply, run:
```
./run_sim.sh
./run_planning.sh
```
the order of the two nodes does not matter.

If you only want to run the planning node for single test and debug, set the param single_test true.

For python, if you want to test the running time of each function:
```
python -m cProfile -s cumulative *.py
```

The corresponding paper "Speed Profile Optimization for Autonomous Vehicles in Dynamic Traffic Scenarios" is submitted to 2018 International Conference on Robotics and Automation(ICRA).
