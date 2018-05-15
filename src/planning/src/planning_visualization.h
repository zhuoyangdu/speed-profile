#ifndef PLANNING_SRC_RRT_PLANNING_VISUALIZATION_H_
#define PLANNING_SRC_RRT_PLANNING_VISUALIZATION_H_

#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace planning {
class PlanningVisualization
{
public:
    PlanningVisualization();
    ~PlanningVisualization();

private:
    void InitPublishers();

private:
    int rate_ = 1;
    ros::NodeHandle nh_;
    ros::Publisher pub_vehicle_state_vis_;
    ros::Publisher pub_obstacle_vis_;
    ros::Publisher pub_trajectory_vis_;

};
}

#endif // PLANNING_SRC_RRT_PLANNING_VISUALIZATION_H_
