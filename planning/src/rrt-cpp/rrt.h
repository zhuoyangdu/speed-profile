#ifndef PLANNING_SRC_RRT_H_
#define PLANNING_SRC_RRT_H_

#include <ros/ros.h>

#include "spline.h"
#include "obstacles.h"
#include "planning/Pose.h"
#include "planning/Trajectory.h"
#include "planning/DynamicObstacle.h"
#include "planning/ObstacleMap.h"

namespace planning{

class Node{
public:
    Node(const double t, const double dis, int s_id=-1){
        time = t;
        distance = dis;
        self_id = s_id;
        velocity = 0;
        cost = 0;
        parent_id = 0;
    }

    void print_node(){
        std::cout << "time:" << time << " distance:" << distance <<
            " vel:" << velocity << " self_id:" << self_id <<
            " parent_id:" << parent_id << " cost:" << cost << std::endl;
    }
public:
    double time;
    double distance;
    double velocity;
    double cost;
    int self_id;
    int parent_id;
};

class RRT{
public:
    RRT();

    void GenerateTrajectory(const planning::Pose& vehicle_state,
                            const planning::ObstacleMap& obstacle_map,
                            const Spline& curve_x,
                            const Spline& curve_y,
                            planning::Trajectory* trajectory);

private:
    std::vector<Node> tree_;
};

}

#endif //PLANNING_SRC_RRT_H_