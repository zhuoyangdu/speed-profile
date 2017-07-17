#include "obstacles.h"

using namespace planning;

Obstacles::Obstacles(){

}

void Obstacles::SetObstacles(const planning::ObstacleMap& obstacle_map){
    obstacles_ = obstacle_map.dynamic_obstacles;
}