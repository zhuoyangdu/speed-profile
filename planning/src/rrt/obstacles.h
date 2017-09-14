#include <ros/ros.h>
#include <deque>
#include <fstream>
#include <cmath>
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

    void InitializeDistanceMap(const planning::Pose& vehicle_state,
                               const Spline& curve_x,
                               const Spline& curve_y,
                               double s0);

    double ReadDistanceMap(const Node& node);

    double ComputeTTC(double node_time, double node_distance,
                      double node_vel,
                      const Spline& curve_x, const Spline& curve_y);

    std::vector<std::vector<double>> ComputeTTCForFixedVel(
                                      double current_path_length,
                                      double node_vel,
                                      const Spline& curve_x,
                                      const Spline& curve_y);

    void ComputeTTCMap(double current_path_length,
                       const Spline& curve_x,
                       const Spline& curve_y);

  private:
    std::vector<planning::DynamicObstacle> obstacles_;
    std::vector<std::vector<double>> distance_map_;
    double danger_distance_;
    double k_risk_;
    double safe_distance_;
    double collision_distance_;
    double t_max_;
    double s_max_;
    double safe_ttc_;
    double NonlinearRisk(double input);
    double init_vehicle_path_length_;
    double epsilon_ = 1e3;
    std::string planning_path_;
    const double kDeltaT = 0.5;
    const double kDeltaS = 2.0;

    void recordDistanceMap();
    double EuclideanDisToObs(double x, double y, double t);
};
} // namespace planning
