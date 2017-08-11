// Copyright [2017] <Copyright ZhuoyangDu>
#ifndef PLANNING_SRC_RRT_COMMON_H_
#define PLANNING_SRC_RRT_COMMON_H_

#include <vector>

namespace planning {

class Node {
 public:
    explicit Node(double t = -1, double dis = -1, int s_id = -1) {
        time = t;
        distance = dis;
        self_id = s_id;
        velocity = 0;
        cost = {0, 0, 0};
        parent_id = 0;
    }

    void print_node() {
        std::cout << "time:" << time << " distance:" << distance <<
            " vel:" << velocity << " self_id:" << self_id <<
            " parent_id:" << parent_id << " cost:" << cost[0] <<
            "," << cost[1] << "," << cost[2] << std::endl;
    }

 public:
    double time;
    double distance;
    double velocity;
    std::vector<double> cost;
    int self_id;
    int parent_id;
};

}  // namespace planning

#endif  // PLANNING_SRC_RRT_COMMON_H_
