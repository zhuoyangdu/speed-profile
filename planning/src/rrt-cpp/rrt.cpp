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
    Node first_node(0, s0, vehicle_state.velocity);
    first_node.self_id = 0;
    first_node.parent_id = -1;
    first_node.print_node();
    tree_ = {first_node};

    int n_sample = 0;
    int n_feasible = 0;
    int n_path = 0;
    double min_cost = 10000;

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
    }
    ends=clock();
    cout << "elapsed time:" << double(ends-start)/CLOCKS_PER_SEC << endl;

    return;
}

void RRT::Extend(Node& sample, Node* new_node, bool* node_valid){
    // Find nearest node.
    Node nearest_node;
    GetNearestNode(sample, &nearest_node, node_valid);
    if(!*node_valid) return;

    // Steer to get new node.
    Steer(sample, nearest_node, new_node);

    bool vertex_feasible = VertexFeasible(nearest_node, *new_node);
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
}