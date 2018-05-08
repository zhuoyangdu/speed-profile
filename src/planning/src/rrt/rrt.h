#ifndef PLANNING_SRC_RRT_H_
#define PLANNING_SRC_RRT_H_

#include <random>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <time.h>

#include "common.h"
#include "spline.h"
#include "obstacles.h"
#include "planning/Pose.h"
#include "planning/Trajectory.h"
#include "planning/DynamicObstacle.h"
#include "planning/ObstacleMap.h"

namespace planning {

class RRT {
  public:
    RRT();

    void GenerateTrajectory(const planning::Pose& vehicle_state,
                            const planning::ObstacleMap& obstacle_map,
                            const Spline& curve_x,
                            const Spline& curve_y,
                            planning::Trajectory* trajectory);

  private:
    double GetGeometryPathLength(double x, double y);

    Node RandomSample(double s0);

    void Extend(Node& sample, Node* new_node, bool* node_valid);

    void GetNearestNode(const Node& sample, Node* nearest_node,
                        bool* node_valid);

    void Steer(const Node& sample, const Node& nearest_node, Node* new_node);

    double ComputeVelocity(const Node& n1, const Node& n2);

    double ComputeAcceleration(const Node& n1, const Node& n2);

    bool VertexFeasible(const Node& parent_node, const Node& child_node);


    std::vector<double> GetNodeCost(const Node& parent_node,
                                    const Node& child_node);

    std::vector<double> GetSingleNodeCost(const Node& node);

    std::deque<Node> GetParentPath(const Node& node);

    std::vector<Node> GetLowerRegion(const Node& node);

    std::vector<Node> GetUpperRegion(const Node& node);

    double WeightingCost(std::vector<double>& cost);

    bool ReachingGoal(const Node& node);

    std::vector<double> GetPathCost(const std::deque<Node>& path);

    double GetPathSmoothness(const std::deque<Node>& path);

    double GetPathVelError(const std::deque<Node>& path);

    void newFile();

    void SendVisualization(const std::deque<Node>& final_path,
                           const Spline& curve_x,
                           const Spline& curve_y);

    void ChooseParent(const Node& nearest_node, Node* new_node);

    void Rewire(Node& new_node);

    std::string int2string(int value);
    std::deque<Node>  PostProcessing(std::deque<Node>& path);
  private:

    Obstacles obstacles;
    std::vector<Node> tree_;
    double max_tree_t_ = 0;

    Spline curve_x_;
    Spline curve_y_;
    int is_rand_ = 0;
    // Config.
    double max_failed_attemptes_;
    double t_max_;
    double s_max_;
    double t_goal_;
    double v_goal_;
    double dt_;
    double max_acc_;
    double max_vel_;
    double kr_;
    double ks_;
    double kv_;
    double lower_range_t_;
    double lower_range_s_;
    double upper_range_t_;
    double upper_range_s_;
    double lower_range_a_;
    double upper_range_a_;

    double k_risk_;
    double danger_distance_;
    double safe_distance_;
    double car_width_;
    double collision_distance_;

    std::string planning_path_;
    std::string file_name_;
    std::ofstream out_file_;
};

}

#endif //PLANNING_SRC_RRT_H_
