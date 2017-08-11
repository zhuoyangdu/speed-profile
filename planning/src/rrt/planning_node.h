#ifndef PLANNING_SRC_PLANNING_NODE_H_
#define PLANNING_SRC_PLANNING_NODE_H_

#include <ros/ros.h>
#include <vector>
#include <memory>
#include <fstream>

#include "rrt.h"
#include "spline.h"
#include "planning/Pose.h"
#include "planning/Trajectory.h"
#include "planning/DynamicObstacle.h"
#include "planning/ObstacleMap.h"

namespace planning {
class PlanningNode {
  public:
    PlanningNode(const ros::NodeHandle& nh);

    void Start();
  private:
    void VehicleStateCallback(const planning::Pose& localize);
    void ObstacleCallback(const planning::ObstacleMap& obstacle_map);
    void GetGeometryPath();
  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_vehicle_state_;
    ros::Subscriber sub_obstacle_;

    bool single_test_ = true;
    double rate_ = 1;

    double vehicle_x0_ = 502.55;
    double vehicle_y0_ = 480.6;
    double vehicle_theta0_ = 0;
    double vehicle_s0_ = 0.6;
    double vehicle_v0_ = 4;
    double obs1_x0_ = 519.5;
    double obs1_y0_ = 502.55;
    double obs1_theta0_ = 4.712;
    double obs1_v0_ = 8;
    double obs2_x0_ = 519.5;
    double obs2_y0_ = 502.55;
    double obs2_theta0_ = 4.712;
    double obs2_v0_ = 8;

    bool obstacle_ready_ = false;
    bool localize_ready_ = false;
    int vehicle_state_queue_size_ = 10;
    int obstacles_queue_size_ = 10;
    planning::Pose vehicle_state_;
    planning::ObstacleMap obstacle_map_;

    std::unique_ptr<RRT> rrt_ptr_;

    Spline curve_x_;
    Spline curve_y_;

    std::string planning_path_;
};
}

#endif //PLANNING_SRC_PLANNING_NODE_H_
