// Copyright [2018] <Zhuoyang Du>

#include "simulation_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    planning::SimulationNode simulation_node(nh);
    simulation_node.Start();
    return 0;
}
