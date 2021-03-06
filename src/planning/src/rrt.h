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
#include <map>

#include "common.h"
#include "spline.h"
#include "obstacles.h"
#include "route.h"
#include "common/Pose.h"
#include "common/Trajectory.h"
#include "common/DynamicObstacle.h"
#include "common/ObstacleMap.h"

#include "common/file_config.h"
#include "planning_config.pb.h"

namespace planning {

class RRT {
  public:
    RRT();

    bool GenerateTrajectory(const common::Pose& vehicle_state,
                            const common::ObstacleMap& obstacle_map,
                            Route* route,
                            common::Trajectory* trajectory);

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

    void PrintTree();

    void PrintPath(const std::vector<Node>& path);

  private:

    Obstacles obstacles;
    std::vector<Node> tree_;
    double max_tree_t_ = 0;

    Route route_;
    Spline curve_x_;
    Spline curve_y_;
    int is_rand_ = 0;

    std::string planning_path_;
    std::string file_name_;
    std::ofstream out_file_;

    planning::PlanningConfig planning_conf_;
    planning::RRTConfig rrt_conf_;

};

}

#endif //PLANNING_SRC_RRT_H_
