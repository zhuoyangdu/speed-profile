#ifndef SIMULATION_SRC_SIMULATION_NODE_H_
#define SIMULATION_SRC_SIMULATION_NODE_H_

#include <ros/ros.h>
#include <vector>

#include "common/Pose.h"
#include "common/Trajectory.h"
#include "common/DynamicObstacle.h"
#include "common/ObstacleMap.h"

#include "sumo/sumo_client.h"

#include "simulation_conf.h"

namespace simulation {
class SimulationNode {
public:
    SimulationNode(const ros::NodeHandle& nh);

    void Start();

private:
    void TrajectoryCallback(const common::Trajectory& trajectory);

private:
    double rate_ = 20;

    ros::NodeHandle nh_;
    ros::Publisher pub_vehicle_state_;
    ros::Publisher pub_obstacle_;
    ros::Subscriber sub_trajectory_;

    bool trajectory_ready_ = false;
    int trajectory_queue_size_ = 10;
    common::Trajectory trajectory_;

    SimulationConf sim_conf_;
    SumoClient sumo_client_;

};

} // namespace simulation

#endif // SIMULATION_SRC_SIMULATION_NODE_H_
