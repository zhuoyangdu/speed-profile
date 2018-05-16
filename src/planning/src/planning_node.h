#ifndef PLANNING_SRC_RRT_PLANNING_NODE_H_
#define PLANNING_SRC_RRT_PLANNING_NODE_H_

#include <ros/ros.h>
#include <vector>
#include <memory>
#include <fstream>

#include "rrt.h"
#include "spline.h"
#include "common/string.h"
#include "common/file_config.h"
#include "planning_visualization.h"
#include "route.h"

#include "common/Pose.h"
#include "common/Trajectory.h"
#include "common/DynamicObstacle.h"
#include "common/ObstacleMap.h"

#include "obstacle_config.pb.h"

namespace planning {
class PlanningNode {
  public:
    PlanningNode(const ros::NodeHandle& nh);

    void Start();

  private:
    void VehicleStateCallback(const common::Pose& localize);
    void ObstacleCallback(const common::ObstacleMap& obstacle_map);
    //void GetGeometryPath();
    void ParamConfig();

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_vehicle_state_;
    ros::Subscriber sub_obstacle_;

    bool single_test_ = true;
    double rate_ =  2;
    std::string road_file_;

    std::vector<common::DynamicObstacle> single_test_obstacles_;
    common::Pose single_test_vehicle_;

    bool obstacle_ready_ = false;
    bool localize_ready_ = false;
    int vehicle_state_queue_size_ = 10;
    int obstacles_queue_size_ = 10;
    common::Pose vehicle_state_;
    common::ObstacleMap obstacle_map_;

    std::unique_ptr<RRT> rrt_ptr_;

    //Spline curve_x_;
    //Spline curve_y_;

    std::string planning_path_;

    Route route_;
    PlanningVisualization planning_vis_;

    ObstacleConfig obstacle_conf_;
};
}

#endif //PLANNING_SRC_RRT_PLANNING_NODE_H_
