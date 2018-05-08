#include <iostream>
#include "ros/ros.h"

#include "sumo/sumo_client.h"

using namespace std;


int main(int argc, char** argv) {

    // Initialize the ROS system.
    ros::init(argc, argv, "simulation_node");

    // Establish this program as a ROS node.
    ros::NodeHandle nh;

    // Send some output as a log message.
    ROS_INFO("This is the simulation node.");

    SumoClient sumo_client;
    sumo_client.Init("localhost", 1338);
    sumo_client.close();

    return 0;
}
