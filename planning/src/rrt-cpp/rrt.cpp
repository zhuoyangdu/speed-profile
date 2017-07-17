#include "rrt.h"
#include <sys/time.h>
#include <vector>
#include <string>

using namespace planning;
RRT::RRT(){

}

//double RRT::GetGeometryPathLength(const Spline& curve_x, const Spline& curve_y)
void RRT::GenerateTrajectory(const planning::Pose& vehicle_state,
                             const planning::ObstacleMap& obstacle_map,
                             const Spline& curve_x,
                             const Spline& curve_y,
                             planning::Trajectory* trajectory) {

    // Initialize obstacles.
    Obstacles obstacles(obstacle_map);

    // Initialize tree.
    double length = 0;
    Node first_node(0, vehicle_state.length, vehicle_state.velocity);
    first_node.print_node();
    return;
}
