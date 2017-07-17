# planning

While the sumo config file changes, use the following command to generate the .net.xml file:
```
netconvert -c crossing.netcfg
```

To run the project:
```
    source devel/setup.bash
    python src/runSim.py
    python src/pathPlan.py
```

The config file of planning is src/rrt/constants.py.

If you want to test the running time of each function:
```
python -m cProfile -s cumulative *.py
```
