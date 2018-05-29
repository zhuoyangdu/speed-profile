#!/bin/zsh
source devel/setup.zsh

rm src/planning/log/*

roslaunch planning planning_node.launch
