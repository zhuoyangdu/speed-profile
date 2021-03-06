#include <ros/ros.h>
#include <deque>
#include <fstream>
#include <cmath>
#include "common.h"
#include "spline.h"
#include "common/Pose.h"
#include "common/Trajectory.h"
#include "common/DynamicObstacle.h"
#include "common/ObstacleMap.h"

#include <common/file_config.h>
#include "planning_debug.pb.h"
#include "planning_config.pb.h"

namespace planning {
class Obstacles {
  public:
    Obstacles();

    void SetObstacles(const common::ObstacleMap& obstacle_map);

    bool CollisionFree(const planning::Node& parent_node,
                       const planning::Node& child_node, const Spline& curve_x,
                       const Spline& curve_y);


    double RiskAssessment(const std::deque<Node>& path,
                          const Spline& curve_x, const Spline& curve_y);

    void InitializeDistanceMap(const common::Pose& vehicle_state,
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

    std::vector<common::DynamicObstacle> GetObstacles() {
      return obstacles_;
    }
  
  private:
    std::vector<common::DynamicObstacle> obstacles_;
    std::vector<std::vector<double>> distance_map_;
    
    /*
    double danger_distance_;
    double k_risk_;
    double safe_distance_;
    double collision_distance_;
    double t_max_;
    double s_max_;
    double t_goal_;
    double max_vel_;
    double safe_ttc_;

    */
    double NonlinearRisk(double input);
    double init_vehicle_path_length_;
    double epsilon_ = 1e3;
    std::string planning_path_;
    const double kDeltaT = 0.1;
    const double kDeltaS = 1.0;

    void recordDistanceMap();
    bool recordDistanceMapProto();
    double EuclideanDisToObs(double x, double y, double t);

    planning::PlanningConfig planning_conf_;
    planning::RRTConfig rrt_conf_;
};
} // namespace planning
