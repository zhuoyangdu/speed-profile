#include <ros/ros.h>

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


private:
    std::vector<planning::DynamicObstacle> obstacles_;
};
} // namespace planning
