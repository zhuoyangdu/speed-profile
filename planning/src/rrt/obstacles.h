#include <ros/ros.h>
#include <deque>
#include <fstream>
#include "common.h"
#include "spline.h"
#include "planning/Pose.h"
#include "planning/Trajectory.h"
#include "planning/DynamicObstacle.h"
#include "planning/ObstacleMap.h"

namespace planning {
class Obstacles {
  public:
    Obstacles();

    void SetObstacles(const planning::ObstacleMap& obstacle_map);

    bool CollisionFree(const planning::Node& parent_node,
                       const planning::Node& child_node, const Spline& curve_x,
                       const Spline& curve_y);


    double RiskAssessment(const std::deque<Node>& path,
                          const Spline& curve_x, const Spline& curve_y);

    void InitializeDistanceMap(const planning::Pose vehicle_state,
                               const Spline& curve_x,
                               const Spline& curve_y,
                               double s0);

    bool DistanceCheck(const Node& node);

  private:
    std::vector<planning::DynamicObstacle> obstacles_;
    std::vector<std::vector<double>> distance_map_;
    double danger_distance_;
    double k_risk_;
    double safe_distance_;
    double t_max_;
    double s_max_;
    double NonlinearRisk(double input);
    double init_vehicle_path_length_;
    std::string planning_path_;
    const double kDeltaT = 0.1;
    const double kDeltaS = 1.0;

    void recordDistanceMap();
};
} // namespace planning
