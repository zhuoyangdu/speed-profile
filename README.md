# planning

While the sumo config file changes, use the following command to generate the .net.xml file:
```
netconvert -c crossing.netcfg
```
To run planning node:
```
source devel/setup.bash
roslaunch planning planning_node.launch
```

To run simulation node:
```
source devel/setup.bash
python planning/src/runSim.py
```
or add -g after runSim.py to open the visualization of sumo.


For python, if you want to test the running time of each function:
```
python -m cProfile -s cumulative *.py
```
