#include "simulation_node.h"
using namespace std;

namespace simulation {
SimulationNode::SimulationNode(const ros::NodeHandle& nh) {
    sumo_client_.Init("localhost", 1337);
}

void SimulationNode::Start() {
    // Publish vehicle state and traffic information.
    pub_vehicle_state_ =
        nh_.advertise<common::Pose>("/simulation/localize", rate_);
    pub_obstacle_ =
        nh_.advertise<common::ObstacleMap>("/simulation/obstacles", rate_);

    // Subscribe trajectory from planning module.
    sub_trajectory_ =
        nh_.subscribe("/planning/trajectory", trajectory_queue_size_,
                      &SimulationNode::TrajectoryCallback, this);

    ros::Rate loop_rate(rate_);

    while(ros::ok()) {
        ros::spinOnce();
        common::Pose vehicle_state;
        common::ObstacleMap obstacle_map;

        pub_obstacle_.publish(obstacle_map);
        pub_vehicle_state_.publish(vehicle_state);

        sumo_client_.GetVehicleList();
        sumo_client_.simulationStep();

        loop_rate.sleep();
    }
}


void SimulationNode::TrajectoryCallback(
        const common::Trajectory& trajectory) {
    trajectory_ = trajectory;
    trajectory_ready_ = true;
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    simulation::SimulationNode simulation_node(nh);
    simulation_node.Start();
    return 0;
}
