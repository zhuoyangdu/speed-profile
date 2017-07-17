#include <ros/ros.h>
#include <deque>
#include "common.h"
#include "spline.h"
#include "planning/Pose.h"
#include "planning/Trajectory.h"
#include "planning/DynamicObstacle.h"
#include "planning/ObstacleMap.h"

namespace planning{
class Obstacles{
public:
    Obstacles();

    void SetObstacles(const planning::ObstacleMap& obstacle_map);

    bool CollisionFree(const planning::Node& parent_node, const planning::Node& child_node, const Spline& curve_x, const Spline& curve_y);


    double RiskAssessment(const std::deque<Node>& path,
                          const Spline& curve_x, const Spline& curve_y);
private:
    std::vector<planning::DynamicObstacle> obstacles_;

    double danger_distance_;
    double k_risk_;
    double safe_distance_;

    double NonlinearRisk(double input);
};
} // namespace planning
