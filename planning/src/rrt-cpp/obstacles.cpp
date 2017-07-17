#include "obstacles.h"

using namespace planning;

Obstacles::Obstacles(const planning::ObstacleMap& obstacle_map){
    obstacles_ = obstacle_map.dynamic_obstacles;
}