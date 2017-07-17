#include "rrt.h"
#include <sys/time.h>
#include <vector>
#include <string>

using namespace std;
using namespace planning;
RRT::RRT(){
    ros::param::get("~rrt/max_failed_attemptes", max_failed_attemptes_);
    ros::param::get("~rrt/t_max", t_max_);
    ros::param::get("~rrt/s_max", s_max_);
    ros::param::get("~rrt/t_goal", t_goal_);
    ros::param::get("~rrt/s_goal", s_goal_);
    ros::param::get("~rrt/v_goal", v_goal_);
    ros::param::get("~rrt/dt", dt_);
    ros::param::get("~rrt/max_acc", max_acc_);
    ros::param::get("~rrt/max_vel", max_vel_);
    ros::param::get("~rrt/kr", kr_);
    ros::param::get("~rrt/ks", ks_);
    ros::param::get("~rrt/kv", kv_);
    ros::param::get("~rrt/lower_range_t", lower_range_t_);
    ros::param::get("~rrt/lower_range_s", lower_range_s_);
    ros::param::get("~rrt/upper_range_t", upper_range_t_);
    ros::param::get("~rrt/upper_range_s", upper_range_s_);
    ros::param::get("~rrt/k_risk", k_risk_);
    ros::param::get("~rrt/danger_distance", danger_distance_);
    ros::param::get("~rrt/safe_distance", safe_distance_);
    ros::param::get("~rrt/car_width", car_width_);
}

void RRT::GenerateTrajectory(const planning::Pose& vehicle_state,
                             const planning::ObstacleMap& obstacle_map,
                             const Spline& curve_x,
                             const Spline& curve_y,
                             planning::Trajectory* trajectory) {
    // Initialize path.
    curve_x_ = curve_x;
    curve_y_ = curve_y;
    double s0 = GetGeometryPathLength(vehicle_state.x, vehicle_state.y);

    // Initialize obstacles.
    obstacles.SetObstacles(obstacle_map);

    // Initialize tree.
    double length = 0;
    Node first_node(0, s0);
    first_node.velocity = vehicle_state.velocity;
    first_node.self_id = 0;
    first_node.parent_id = -1;
    first_node.print_node();
    tree_ = {first_node};

    int n_sample = 0;
    int n_feasible = 0;
    int n_path = 0;
    double min_cost = 10000;
    std::deque<Node> min_path;

    clock_t start,ends;
    start=clock();
    while(n_sample < max_failed_attemptes_){
        // Sample.
        n_sample = n_sample + 1;
        Node sample = RandomSample(s0);

        // Extend.
        bool node_valid;
        Node new_node(-1,-1,-1);
        Extend(sample, &new_node, &node_valid);
        // sample.print_node();

        if(node_valid){
            n_feasible = n_feasible + 1;
            if(ReachingGoal(new_node)){
                n_path = n_path + 1;
                std::deque<Node> path = GetParentPath(new_node);
                std::vector<double> path_cost = GetPathCost(path);
                double cost_sum = path_cost[0] + path_cost[1] + path_cost[2];
                if(cost_sum < min_cost){
                    min_path = path;
                    min_cost = cost_sum;
                }
                // cout << "found " << n_path << "paths" << endl;
                // PrintNodes(path);
                if(n_path > 5){
                    cout << "The final path is: " << endl;
                    PrintNodes(min_path);
                    cout << "The cost of the final path is:" << min_cost << endl;
                    break;
                }
            }

        }
    }

    cout << "total attemps:" << n_sample << ", feasible sample:" << n_feasible << endl;
    // cout << "tree:" << endl;
    // PrintNodes(tree_);
    ends=clock();
    cout << "elapsed time:" << double(ends-start)/CLOCKS_PER_SEC << endl;

    if(n_path > 0){
        std::vector<planning::Pose> poses;
        for(int i = 0; i < min_path.size(); i++){
            planning::Pose pose;
            pose.timestamp = min_path[i].time + vehicle_state.timestamp;
            pose.velocity = min_path[i].velocity;
            pose.length = min_path[i].distance;
            pose.x = curve_x_(pose.length);
            pose.y = curve_y_(pose.length);
            poses.push_back(pose);
        }
        trajectory->poses = poses;
    }else{
        ROS_ERROR("No path found.");
    }
    return;
}

void RRT::Extend(Node& sample, Node* new_node, bool* node_valid){
    // Find nearest node.
    Node nearest_node;
    GetNearestNode(sample, &nearest_node, node_valid);
    if(!*node_valid){
        return;
    }

    // std::cout << "sample:"<<endl;
    // sample.print_node();
    // std::cout << "nearest_node:" << endl;
    // nearest_node.print_node();

    // Steer to get new node.
    Steer(sample, nearest_node, new_node);

    // std::cout << "new node:" << endl;
    // new_node->print_node();

    bool vertex_feasible = VertexFeasible(nearest_node, *new_node);
    if(vertex_feasible){
        *node_valid = true;
        Node min_node = nearest_node;
        std::vector<double> cost_min = GetNodeCost(min_node, *new_node);
        // cout << "cost of nearest node:" << endl;
        // PrintCost(cost_min);

        std::vector<Node> near_region = GetLowerRegion(*new_node);
        // cout << "near lower region:" << endl;
        // PrintNodes(near_region);

        for(int i = 0; i < near_region.size(); i++){
            Node near_node = near_region[i];
            vertex_feasible = VertexFeasible(near_node, *new_node);
            if(vertex_feasible){
                std::vector<double> cost_near = GetNodeCost(near_node, *new_node);
                // cout << "cost_near:" << endl;
                // PrintCost(cost_near);
                // cout << "weighting near:" << WeightingCost(cost_near) << endl;
                // cout << "weighting min:" << WeightingCost(cost_min) << endl;
                if(WeightingCost(cost_near) < WeightingCost(cost_min)){
                    // PrintCost(cost_min);
                    // PrintCost(cost_near);
                    cost_min = cost_near;
                    min_node = near_node;
                }
            }
        }
        new_node->parent_id = min_node.self_id;
        new_node->self_id = tree_.size();
        new_node->velocity = ComputeVelocity(min_node, *new_node);
        new_node->cost = cost_min;
        // cout << "Add a new node to the tree." << endl;
        // new_node->print_node();
        tree_.push_back(*new_node);

        near_region = GetUpperRegion(*new_node);
        // cout << "near upper region:" << endl;
        // PrintNodes(near_region);

        for(int i = 0; i < near_region.size(); i++){
            Node near_node = near_region[i];
            vertex_feasible = VertexFeasible(*new_node, near_node);
            if(vertex_feasible){
                std::vector<double> cost_near = near_node.cost;
                std::vector<double> cost_new = GetNodeCost(*new_node, near_node);
                if(WeightingCost(cost_near) > WeightingCost(cost_new)){
                    near_node.parent_id = new_node->self_id;
                    near_node.velocity = ComputeVelocity(*new_node, near_node);
                    near_node.cost = cost_new;
                    tree_[near_node.self_id] = near_node;
                    cout << "rewire tree." << endl;
                }
            }
        }
    }else{
        // std::cout << "no feasible!" << endl;
        *node_valid = false;
        return;
    }
}

double RRT::WeightingCost(std::vector<double>& cost){
    return kr_ * cost[0] + ks_ * cost[1] + kv_ * cost[2];
}

Node RRT::RandomSample(double s0){
    double sample_t = (double) rand()/RAND_MAX * t_max_;
    double sample_s = (double) rand()/RAND_MAX * s_max_ + s0;
    Node sample(sample_t, sample_s);
    return sample;
}

int getMinIndex(const std::vector<double>& v){
    std::vector<int>::iterator result;
    // result = std::min_element(v.begin(), v.end());
    int index= std::distance(v.begin(), std::min_element(v.begin(), v.end()));
    // std::cout << "max element at: " << index << endl;
    return index;
}

void RRT::GetNearestNode(const Node& sample,
                         Node* nearest_node,
                         bool* node_valid){
    std::vector<double> dist;
    for (int i = 0; i < tree_.size(); i++){
        double delta_s = sample.distance - tree_[i].distance;
        double delta_t = sample.time - tree_[i].time;
        // cout << "delta_s:"<< delta_s << " delta_t:"<< delta_t << endl;
        if(delta_s < 0 || delta_t < 0){
            dist.push_back(10000);
        } else {
            double vel = delta_s / delta_t;
            double acc = (tree_[i].velocity - vel) / delta_t;
            if(vel > max_vel_ || fabs(acc) > max_acc_){
                dist.push_back(10000);
            } else {
                dist.push_back(sqrt(pow(delta_s/s_max_,2) + pow(delta_t/t_max_,2)));
            }
        }
        // cout << "dist:" << dist[i] << endl;
    }
    int min_index = getMinIndex(dist);
    if (dist[min_index] >= 10000){
        *node_valid = false;
    } else {
        *node_valid = true;
    }
    *nearest_node = tree_[min_index];
}

void RRT::Steer(const Node& sample, const Node& nearest_node, Node* new_node){
    double k = ComputeVelocity(sample, nearest_node);
    new_node->time = nearest_node.time + dt_;
    new_node->distance = nearest_node.distance + k * dt_;
    new_node->velocity = k;
    new_node->self_id = tree_.size();
}

double RRT::ComputeVelocity(const Node& n1, const Node& n2){
    return (n1.distance - n2.distance) / (n1.time - n2.time);
}

double RRT::ComputeAcceleration(const Node& n1, const Node& n2){
    double vel = ComputeVelocity(n1, n2);
    return (n1.velocity - vel) / (n1.time - n2.time);
}

double RRT::GetGeometryPathLength(double x, double y){
    double s = 0;
    double d = 0;
    Spline::getClosestPointOnCurve(curve_x_, curve_y_, x, y, &s, &d);
    double ref_x = curve_x_(s);
    double ref_y = curve_y_(s);
    //cout << "ref_x:" << ref_x << ", ref_y:" << ref_y << ", length:" << s 
    //    <<",d:" << d << " x:" << x << ", y:" << y << endl;
    return s;
}

bool RRT::VertexFeasible(const Node& parent_node, const Node& child_node){
    if(parent_node.time >= child_node.time
        || parent_node.distance > child_node.distance){
        ROS_ERROR("error in find nodes.");
        return false;
    }
    double vel = ComputeVelocity(parent_node, child_node);
    if(vel > max_vel_){
        return false;
    }
    double acc = ComputeAcceleration(parent_node, child_node);
    if(fabs(acc) > max_acc_){
        return false;
    }
    bool collision_free = obstacles.CollisionFree(parent_node, child_node, curve_x_, curve_y_);
    if(!collision_free){
        return false;
    }
    return true;
}

std::deque<Node> RRT::GetParentPath(const Node& node){
    Node child_node = node;
    std::deque<Node> path = {child_node};
    while(child_node.parent_id != -1){
        Node parent_node = tree_[child_node.parent_id];
        path.push_front(parent_node);
        child_node = parent_node;
    }
    return path;
}

std::vector<double> RRT::GetPathCost(const std::deque<Node>& path){
    double risk = obstacles.RiskAssessment(path, curve_x_, curve_y_);
    double smoothness = GetPathSmoothness(path);
    double e_vel = GetPathVelError(path);
    std::vector<double> cost = {risk, smoothness, e_vel};
    return cost;
}

double RRT::GetPathSmoothness(const std::deque<Node>& path){
    double sum_acc = 0;
    for(int i = 1; i < path.size(); i++){
        double acc = (path[i].velocity - path[i-1].velocity) /
                     (path[i].time - path[i-1].time);
        sum_acc = sum_acc + acc;
    }
    return sum_acc;
}

double RRT::GetPathVelError(const std::deque<Node>& path){
    double ev = 0;
    for(int i = 0; i < path.size(); i++){
        ev = ev + fabs(path[i].velocity - v_goal_);
    }
    return ev;
}

std::vector<double> RRT::GetNodeCost(const Node& parent_node, const Node& child_node){
    std::deque<Node> path = GetParentPath(parent_node);

    double risk = obstacles.RiskAssessment(path, curve_x_, curve_y_);
    double smoothness = GetNodeSmooth(parent_node, child_node);
    double e_vel = GetNodeVelError(parent_node, child_node);
    std::vector<double> cost = {risk, smoothness, e_vel};
    return cost;
}

double RRT::GetNodeSmooth(const Node& parent_node, const Node& child_node){
    double child_vel = ComputeVelocity(parent_node, child_node);
    return fabs(parent_node.velocity - child_vel);
}

double RRT::GetNodeVelError(const Node& parent_node, const Node& child_node){
    double child_vel = ComputeVelocity(parent_node, child_node);
    return fabs(child_vel-v_goal_);
}

std::vector<Node> RRT::GetLowerRegion(const Node& node){
    double min_time = node.time - lower_range_t_;
    double min_distance = node.distance - lower_range_s_;
    std::vector<Node> near_region;
    for(int i = 0; i < tree_.size(); i++){
        if(tree_[i].time < node.time
            && tree_[i].time > min_time
            && tree_[i].distance < node.distance
            && tree_[i].distance > min_distance){
            near_region.push_back(tree_[i]);
        }
    }
    return near_region;
}

std::vector<Node> RRT::GetUpperRegion(const Node& node){
    double max_time = node.time + upper_range_t_;
    double max_distance = node.time + upper_range_s_;
    std::vector<Node> near_region;
    for(int i = 0; i < tree_.size(); i++){
        if(tree_[i].time > node.time
            && tree_[i].time < max_time
            && tree_[i].distance > node.distance
            && tree_[i].distance < max_distance){
            near_region.push_back(tree_[i]);
        }
    }
    return near_region;
}

void RRT::PrintNodes(std::vector<Node>& nodes){
    cout << "size:" << nodes.size() << endl;
    for(int i = 0; i < nodes.size(); i++){
        nodes[i].print_node();
    }
}

void RRT::PrintNodes(std::deque<Node>& nodes){
    cout << "size:" << nodes.size() << endl;
    for(int i = 0; i < nodes.size(); i++){
        nodes[i].print_node();
    }
}

void RRT::PrintCost(std::vector<double>& cost){
    cout << "cost:" << "risk:" << cost[0] << ",smoothness:" << cost[1]
        << ",vel error:" << cost[2] << endl;
}

bool RRT::ReachingGoal(const Node& node){
    if(abs(node.time - t_goal_) < 0.3){
        return true;
    }
    return false;
}