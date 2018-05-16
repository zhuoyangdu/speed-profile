#ifndef PLANNING_SRC_RRT_PLANNING_VISUALIZATION_H_
#define PLANNING_SRC_RRT_PLANNING_VISUALIZATION_H_

#include <iostream>
#include <memory>
#include <fstream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <common/string.h>

namespace planning {
class PlanningVisualization
{
public:
    PlanningVisualization();

    void PubEnv();

private:
    void InitPublishers();

    void PaintMap();

    std::vector<std::vector<double>> GetMapFromFile();

private:
    std::string planning_path_;

    int rate_ = 1;
    ros::NodeHandle nh_;
    ros::Publisher pub_vehicle_state_vis_;
    ros::Publisher pub_obstacle_vis_;
    ros::Publisher pub_trajectory_vis_;
    ros::Publisher pub_map_;

};
}

#endif // PLANNING_SRC_RRT_PLANNING_VISUALIZATION_H_
