#include "rrt.h"
#include <sys/time.h>
#include <vector>
#include <string>

using namespace std;
using namespace planning;
RRT::RRT() {
    ros::param::get("~rrt/max_failed_attemptes", max_failed_attemptes_);
    ros::param::get("~rrt/t_max", t_max_);
    ros::param::get("~rrt/s_max", s_max_);
    ros::param::get("~rrt/t_goal", t_goal_);
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
    ros::param::get("~rrt/lower_range_a", lower_range_a_);
    ros::param::get("~rrt/upper_range_a", upper_range_a_);
    ros::param::get("~rrt/k_risk", k_risk_);
    ros::param::get("~rrt/danger_distance", danger_distance_);
    ros::param::get("~rrt/collision_distance", collision_distance_);
    ros::param::get("~rrt/safe_distance", safe_distance_);
    ros::param::get("~rrt/car_width", car_width_);
    ros::param::get("~planning_path", planning_path_);
}

void RRT::newFile() {
    time_t t = std::time(NULL);
    struct tm * now = std::localtime(&t);
    file_name_ = planning_path_ + "/log/rrt.txt";
    std::ofstream out_file_(file_name_.c_str());
    if (!out_file_) {
        ROS_INFO("no file!");
    }
    out_file_.close();
}

void RRT::GenerateTrajectory(const planning::Pose& vehicle_state,
                             const planning::ObstacleMap& obstacle_map,
                             const Spline& curve_x,
                             const Spline& curve_y,
                             planning::Trajectory* trajectory) {
    cout << "Start planning!" << endl;
    cout << "Vehicle state:" << vehicle_state.x << "," << vehicle_state.y << ","
         << vehicle_state.theta << "," << vehicle_state.velocity << endl;
    cout << "Obstacles:" << endl;
    for (int i = 0; i < obstacle_map.dynamic_obstacles.size(); i++) {
        cout << obstacle_map.dynamic_obstacles[i].id << ","
             << obstacle_map.dynamic_obstacles[i].x << ","
             << obstacle_map.dynamic_obstacles[i].y << ","
             << obstacle_map.dynamic_obstacles[i].theta << ","
             << obstacle_map.dynamic_obstacles[i].velocity << endl;
    }

    newFile();
    // record.
    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
    for (int i = 0; i < obstacle_map.dynamic_obstacles.size(); i++) {
        planning::DynamicObstacle obs = obstacle_map.dynamic_obstacles[i];
        out_file_ << "obstacle\t" << obs.timestamp << "\t" <<
                  obs.id << "\t" << obs.x << "\t" << obs.y << "\t"
                  << obs.theta << "\t" << obs.velocity << "\n";
    }
    out_file_ << "vehicle_state\t" << vehicle_state.timestamp << "\t"
              << vehicle_state.x << "\t" << vehicle_state.y << "\t"
              << vehicle_state.theta << "\t" << vehicle_state.velocity << "\n";
    //out_file_.close();

    clock_t temp_t = clock();
    // Initialize path.
    curve_x_ = curve_x;
    curve_y_ = curve_y;
    double s0 = GetGeometryPathLength(vehicle_state.x, vehicle_state.y);

    // Initialize obstacles.
    obstacles.SetObstacles(obstacle_map);
    obstacles.InitializeDistanceMap(vehicle_state, curve_x_, curve_y_, s0);
    obstacles.ComputeTTCMap(s0, curve_x_, curve_y_);
    cout << "Initial time:" << double(clock() - temp_t) / CLOCKS_PER_SEC << endl;

    // Initialize tree.
    double length = 0;
    Node first_node(0, s0);
    first_node.velocity = vehicle_state.velocity;
    first_node.acceleration = vehicle_state.acceleration;
    first_node.self_id = 0;
    first_node.parent_id = -1;
    tree_ = {first_node};

    int n_sample = 0;
    int n_feasible = 0;
    int n_path = 0;
    double min_cost = 10000;
    un_vel = 0;
    un_acc = 0;
    un_collision = 0;
    std::deque<Node> min_path;

    clock_t start, ends;
    start = clock();
    srand(time(NULL));
    while (n_sample < max_failed_attemptes_) {
        // Sample.
        n_sample = n_sample + 1;
        Node sample = RandomSample(s0);
        if (obstacles.ReadDistanceMap(sample) < collision_distance_) {
            continue;
        }
        std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
        // out_file_ << "sample\t" << sample.time << "\t" << sample.distance << "\n";
        // out_file_.close();

        // Extend.
        bool node_valid;
        Node new_node(-1, -1, -1);
        Extend(sample, &new_node, &node_valid);
        // sample.print_node();
        if (node_valid) {
            n_feasible = n_feasible + 1;
            if (ReachingGoal(new_node)) {
                n_path = n_path + 1;
                std::deque<Node> path = GetParentPath(new_node);
                std::vector<double> path_cost = GetPathCost(path);
                double cost_sum = WeightingCost(path_cost);
                // cout << "current path cost: " << cost_sum << ", min cost:" << min_cost <<
                //     endl;
                if (cost_sum < min_cost) {
                    min_path = path;
                    min_cost = cost_sum;

                    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
                    out_file_ << "path\n";
                    for (int i = 0; i < path.size(); i++) {
                        out_file_ << path[i].time << "\t" << path[i].distance << "\t" <<
                                  path[i].velocity << "\n";
                    }
                    out_file_ << "end_path\n";
                    out_file_ << "path_cost:" << cost_sum << "," << path_cost[0]*kr_ << ","
                              << path_cost[1]*ks_ << "," << path_cost[2]*kv_ << "\n";
                    out_file_.close();
                }
                if (n_path > 500) {
                    cout << "The final path is:" << endl;
                    PrintNodes(min_path);
                    cout << "The cost of the final path is:" << min_cost << endl;
                    break;
                }
            }
        }
    }

    // std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
    out_file_ << "final_path\n";
    for (int i = 0; i < min_path.size(); i++) {
        out_file_ << min_path[i].time << "\t" << min_path[i].distance << "\t" <<
                  min_path[i].velocity << "\n";
    }
    out_file_ << "end_path\n";
    // std::ofstream out_file_(file_name_.c_str(), std::ios::in|std::ios::app);
    out_file_ << "tree\n";
    for (int i = 0; i < tree_.size(); i++) {
        out_file_ << tree_[i].time << "\t" << tree_[i].distance << "\t" <<
                  tree_[i].velocity << "\t" << tree_[i].parent_id << "\n";
    }
    out_file_ << "end_tree\n";
    out_file_.close();
    cout << "total attemps:" << n_sample << ", feasible sample:" << n_feasible <<
         endl;
    cout << "unvel:" << un_vel << ", unacc:" << un_acc << ", un_col:" <<
         un_collision << endl;
    cout << "choose_par_un_feasible:" << choose_par_un_feasible << "," <<
         "rewire_un_feasible:" << rewire_un_feasible << endl;
    ends = clock();
    cout << "elapsed time:" << double(ends - start) / CLOCKS_PER_SEC << endl;

    cout << "module time:" << endl;
    cout << "nearest time:" << time_nearest_ << endl;
    cout << "steer time:" << time_steer_ << endl;
    cout << "feasible time:" << time_feasible_ << endl;
    cout << "choose parent time:" << time_choose_parent_ << endl;
    cout << "rewire time:" << time_rewire_ << endl;

    if (n_path > 0) {
        std::deque<Node> smoothing_path = PostProcessing(min_path);
        SendVisualization(smoothing_path, curve_x_, curve_y_);

        std::vector<planning::Pose> poses;
        for (int i = 0; i < min_path.size(); i++) {
            planning::Pose pose;
            pose.timestamp = min_path[i].time + vehicle_state.timestamp;
            pose.velocity = min_path[i].velocity;
            pose.length = min_path[i].distance;
            pose.x = curve_x_(pose.length);
            pose.y = curve_y_(pose.length);
            poses.push_back(pose);
        }
        trajectory->poses = poses;
    } else {
        ROS_ERROR("No path found.");
    }
    return;
}


void RRT::Extend(Node& sample, Node* new_node, bool* node_valid) {

    // Find nearest node.
    Node nearest_node;
    clock_t temp_t = clock();
    GetNearestNode(sample, &nearest_node, node_valid);
    time_nearest_ += double(clock() - temp_t) / CLOCKS_PER_SEC;

    if (!*node_valid) {
        return;
    }

    int n_collision_sample = 0;

    // Steer to get new node.
    temp_t = clock();
    Steer(sample, nearest_node, new_node);
    time_steer_ += double(clock() - temp_t) / CLOCKS_PER_SEC;

    if (new_node->time > t_goal_) {
        n_collision_sample += 1;
        *node_valid = false;
        return;
    }

    temp_t = clock();
    bool vertex_feasible = VertexFeasible(nearest_node, *new_node);
    time_feasible_ += double(clock() - temp_t) / CLOCKS_PER_SEC;

    if (vertex_feasible) {
        *node_valid = true;
        clock_t temp_t = clock();
        ChooseParent(nearest_node, new_node);
        time_choose_parent_ += double(clock() - temp_t) / CLOCKS_PER_SEC;
        temp_t = clock();
        Rewire(*new_node);
        time_rewire_ += double(clock() - temp_t) / CLOCKS_PER_SEC;
    } else {
        *node_valid = false;
        return;
    }
}

void RRT::Rewire(Node& new_node) {
    std::vector<Node> near_region = GetUpperRegion(new_node);
    for (int i = 0; i < near_region.size(); i++) {
        Node near_node = near_region[i];
        bool vertex_feasible = VertexFeasible(new_node, near_node);
        if (!vertex_feasible) rewire_un_feasible += 1;
        if (vertex_feasible) {
            // std::vector<double> cost_near = near_node.cost;
            std::vector<double> cost_near = GetSingleNodeCost(near_node);
            std::vector<double> cost_new = GetNodeCost(new_node, near_node);
            if (WeightingCost(cost_near) > WeightingCost(cost_new)) {
                int previous_parent_id = near_node.parent_id;
                Node previous_parent = tree_[near_node.parent_id];
                near_node.parent_id = new_node.self_id;
                near_node.velocity = ComputeVelocity(new_node, near_node);
                near_node.acceleration = ComputeAcceleration(new_node, near_node);
                tree_[near_node.self_id] = near_node;
                tree_[new_node.self_id].children_id.push_back(near_node.self_id);
                vector<int>::iterator iter = find(previous_parent.children_id.begin(),
                                                  previous_parent.children_id.end(), near_node.self_id);
                if (iter != previous_parent.children_id.end())
                    previous_parent.children_id.erase(iter);
                tree_[previous_parent_id] = previous_parent;
                for(int k = 0; k < near_node.children_id.size(); k++){
                    int child_index = near_node.children_id[k];
                    tree_[child_index].acceleration = ComputeAcceleration(near_node, tree_[child_index]);
                }
            }
        }
    }
}
void RRT::ChooseParent(const Node& nearest_node, Node* new_node) {
    Node min_node = nearest_node;
    std::vector<double> cost_min = GetNodeCost(min_node, *new_node);

    std::vector<Node> near_region = GetLowerRegion(*new_node);
    for (int i = 0; i < near_region.size(); i++) {
        Node near_node = near_region[i];
        bool vertex_feasible = VertexFeasible(near_node, *new_node);
        if (!vertex_feasible) choose_par_un_feasible += 1;
        if (vertex_feasible) {
            std::vector<double> cost_near = GetNodeCost(near_node, *new_node);
            if (WeightingCost(cost_near) < WeightingCost(cost_min)) {
                cost_min = cost_near;
                min_node = near_node;
            }
        }
    }
    new_node->parent_id = min_node.self_id;
    new_node->self_id = tree_.size();
    new_node->velocity = ComputeVelocity(min_node, *new_node);
    new_node->acceleration = ComputeAcceleration(min_node, *new_node);
    // new_node->cost = cost_min;
    tree_.push_back(*new_node);
    tree_[min_node.self_id].children_id.push_back(new_node->self_id);
    if (new_node->time > max_tree_t_) {
        max_tree_t_ = new_node->time;
    }
}

double RRT::WeightingCost(std::vector<double>& cost) {
    double w = kr_ * cost[0] + ks_ * cost[1] + kv_ * cost[2];
    return w;
}

Node RRT::RandomSample(double s0) {
    double sample_t = 0;
    double sample_s = 0;
    double sample_s_range = s_max_ < t_max_ * max_vel_ ? s_max_ : t_max_ *
                            max_vel_;
    sample_t = (double) rand() / RAND_MAX * t_max_;
    double s_range = sample_t * max_vel_;
    sample_s = (double) rand() / RAND_MAX * s_range + s0;
    Node sample(sample_t, sample_s);
    return sample;
}

int getMinIndex(const std::vector<double>& v) {
    std::vector<int>::iterator result;
    int index = std::distance(v.begin(), std::min_element(v.begin(), v.end()));
    return index;
}

void RRT::GetNearestNode(const Node& sample,
                         Node* nearest_node,
                         bool* node_valid) {
    std::vector<double> dist;
    for (int i = 0; i < tree_.size(); i++) {
        double delta_s = sample.distance - tree_[i].distance;
        double delta_t = sample.time - tree_[i].time;

        if (delta_s < -0.5 || delta_t < 0) {
            dist.push_back(10000);
        } else {
            double vel = delta_s / delta_t;
            if (delta_s <= 0) {
                vel = 0;
            }
            double acc = (tree_[i].velocity - vel) / delta_t;
            if (vel > max_vel_ || fabs(acc) > max_acc_) {
                dist.push_back(10000);
            } else {
                dist.push_back(fabs(acc) + delta_t);
            }
        }
    }
    int min_index = getMinIndex(dist);
    if (dist[min_index] >= 10000) {
        *node_valid = false;
    } else {
        *node_valid = true;
    }
    *nearest_node = tree_[min_index];
}

void RRT::Steer(const Node& sample, const Node& nearest_node,
                Node* new_node) {

    double k = ComputeVelocity(sample, nearest_node);
    if (k <= 0) k = 0;
    new_node->time = nearest_node.time + dt_;
    new_node->distance = nearest_node.distance + k * dt_;
    new_node->velocity = k;
    new_node->self_id = tree_.size();
}

double RRT::ComputeVelocity(const Node& n1, const Node& n2) {
    return (n1.distance - n2.distance) / (n1.time - n2.time);
}

double RRT::ComputeAcceleration(const Node& n1, const Node& n2) {
    double vel = ComputeVelocity(n1, n2);
    return (n1.velocity - vel) / (n1.time - n2.time);
}

double RRT::GetGeometryPathLength(double x, double y) {
    double s = 0;
    double d = 0;
    Spline::getClosestPointOnCurve(curve_x_, curve_y_, x, y, &s, &d);
    double ref_x = curve_x_(s);
    double ref_y = curve_y_(s);
    return s;
}

bool RRT::VertexFeasible(const Node& parent_node, const Node& child_node) {
    if (parent_node.time >= child_node.time
            || parent_node.distance > child_node.distance) {
        ROS_ERROR("error in find nodes.");
        return false;
    }
    double vel = ComputeVelocity(parent_node, child_node);
    if (vel > max_vel_) {
        un_vel += 1;
        return false;
    }

    double acc = ComputeAcceleration(parent_node, child_node);
    if (fabs(acc) > max_acc_) {
        un_acc += 1;
        return false;
    }
    bool collision_free = obstacles.CollisionFree(parent_node, child_node,
                          curve_x_, curve_y_);
    if (!collision_free) {
        un_collision += 1;
        return false;
    }
    return true;
}

std::deque<Node> RRT::GetParentPath(const Node& node) {
    Node child_node = node;
    std::deque<Node> path = {child_node};
    while (child_node.parent_id != -1) {
        Node parent_node = tree_[child_node.parent_id];
        path.push_front(parent_node);
        child_node = parent_node;
    }
    return path;
}

std::vector<double> RRT::GetPathCost(const std::deque<Node>& path) {
    double risk = obstacles.RiskAssessment(path, curve_x_, curve_y_);
    double smoothness = GetPathSmoothness(path);
    double e_vel = GetPathVelError(path);
    std::vector<double> cost = {risk, smoothness, e_vel};
    return cost;
}

double RRT::GetPathSmoothness(const std::deque<Node>& path) {
    double sum_abs_acc = 0;
    double sum_acc = 0;
    std::vector<double> vector_acc;
    for (int i = 0; i < path.size(); i++) {
        // double acc = (path[i].velocity - path[i - 1].velocity) /
        //             (path[i].time - path[i - 1].time);
        double acc = path[i].acceleration;
        vector_acc.push_back(acc);
        sum_acc = sum_acc + acc;
        sum_abs_acc = sum_abs_acc + fabs(acc);
    }

    // double min_acc = fabs((path.front().velocity - path.back().velocity)
    //                      / (path.front().time - path.back().time));
    double average_acc = sum_acc / vector_acc.size();
    double variance_acc = 0;
    for (int i = 0; i < vector_acc.size(); i++) {
        variance_acc = variance_acc + abs(vector_acc[i] - average_acc);
    }
    variance_acc = abs(variance_acc) / vector_acc.size() * 10;

    return variance_acc;
}

double RRT::GetPathVelError(const std::deque<Node>& path) {
    double ev = 0;
    for (int i = 0; i < path.size(); i++) {
        ev = ev + fabs(path[i].velocity - v_goal_);
    }
    return ev / path.size();
}

std::vector<double> RRT::GetNodeCost(const Node& parent_node,
                                     const Node& child_node) {
    std::deque<Node> path = GetParentPath(parent_node);
    Node new_child = child_node;
    new_child.velocity = ComputeVelocity(parent_node, child_node);
    path.push_back(new_child);
    std::vector<double> cost = GetPathCost(path);
    return cost;
}

std::vector<double> RRT::GetSingleNodeCost(const Node& node){
    std::deque<Node> path = GetParentPath(node);
    std::vector<double> cost = GetPathCost(path);
    return cost;
}

std::vector<Node> RRT::GetLowerRegion(const Node& node) {
    std::vector<Node> near_region;
    for (int i = 0; i < tree_.size(); i++) {
        if (tree_[i].time < node.time
                && tree_[i].distance < node.distance
                && node.time - tree_[i].time < 1) {
            double vel = ComputeVelocity(tree_[i], node);
            double child_acc = ComputeAcceleration(tree_[i], node);
            double parent_acc = tree_[i].acceleration;
            double delta_acc = parent_acc - child_acc;
            if (fabs(delta_acc) < lower_range_a_ && fabs(vel) < max_vel_) {
                near_region.push_back(tree_[i]);
            }
        }
    }
    return near_region;
}

std::vector<Node> RRT::GetUpperRegion(const Node& node) {
    std::vector<Node> near_region;
    for (int i = 0; i < tree_.size(); i++) {
        if (tree_[i].time > node.time
                && tree_[i].distance > node.distance
                && tree_[i].time - node.time < 1) {
            double vel = ComputeVelocity(node, tree_[i]);
            double parent_acc = node.acceleration;
            double child_acc = ComputeAcceleration(node, tree_[i]);
            double delta_acc = parent_acc - child_acc;
            if (fabs(delta_acc) < lower_range_a_ && vel < max_vel_) {
                near_region.push_back(tree_[i]);
            }
        }
    }
    return near_region;
}


void RRT::PrintNodes(std::vector<Node>& nodes) {
    cout << "size:" << nodes.size() << endl;
    for (int i = 0; i < nodes.size(); i++) {
        std::cout << "t:" << nodes[i].time << " s:" << nodes[i].distance
            << " vel:" << nodes[i].velocity << " acc:" << nodes[i].acceleration;
        std::vector<double> cost = GetSingleNodeCost(nodes[i]);
        cout << " cost:" << cost[0] << ", " << cost[1] << ", " << cost[2] << endl;
    }
}

void RRT::PrintNodes(std::deque<Node>& nodes) {
    cout << "size:" << nodes.size() << endl;
    for (int i = 0; i < nodes.size(); i++) {
        std::cout << "t:" << nodes[i].time << " s:" << nodes[i].distance
            << " vel:" << nodes[i].velocity << " acc:" << nodes[i].acceleration;
        std::vector<double> cost = GetSingleNodeCost(nodes[i]);
        cout << " cost:" << cost[0] << ", " << cost[1] << ", " << cost[2] << endl;
    }
}

void RRT::PrintCost(std::vector<double>& cost) {
    cout << "cost:" << "risk:" << kr_* cost[0] << ",smoothness:" << ks_ * cost[1]
         << ",vel error:" << kv_ * cost[2] << endl;
}

bool RRT::ReachingGoal(const Node& node) {
    if (abs(node.time - t_goal_) < 0.3) {
        return true;
    }
    return false;
}

std::string RRT::int2string(int value) {
    std::stringstream ss;
    if (value < 10) {
        ss << 0 << value;
    } else {
        ss << value;
    }
    return ss.str();
}

std::deque<Node>  RRT::PostProcessing(std::deque<Node>& path) {
    std::deque<Node> full_path;
    for (int i = 0; i < path.size() - 1; i++) {
        double n = 10 * (path[i + 1].time - path[i].time);
        // TODO: figure out the error.
        //cout << n << "path time:" << path[i+1].time << "," << path[i].time << endl;
        //cout << "n:" << n << "int n" << static_cast<int>(n) << endl;
        for (int k = 0; k < n - 0.5; k++) {
            Node node;
            node.time = path[i].time + k * dt_;
            node.distance = path[i].distance + path[i + 1].velocity * dt_ * k;
            full_path.push_back(node);
        }
    }
    full_path.push_back(path.back());

    std::deque<Node> smoothing_path;
    smoothing_path.push_back(full_path[0]);
    Node node = full_path[1];
    node.distance = (full_path[0].distance + full_path[1].distance +
                     full_path[2].distance) / 3.0;
    smoothing_path.push_back(node);

    for (int i = 2; i < full_path.size() - 2; i++) {
        Node node = full_path[i];
        node.distance = (full_path[i - 2].distance + full_path[i - 1].distance +
                         full_path[i].distance +
                         full_path[i + 1].distance + full_path[i + 2].distance) / 5.0;
        smoothing_path.push_back(node);
    }

    int sz = full_path.size();
    node  = full_path[sz - 2];
    node.distance = (full_path[sz - 3].distance + full_path[sz - 2].distance +
                     full_path[sz - 1].distance) /
                    3.0;
    smoothing_path.push_back(node);
    node = full_path[sz - 1];
    smoothing_path.push_back(node);

    smoothing_path[0].velocity = path[0].velocity;
    for (int i = 1; i < smoothing_path.size(); i++) {
        smoothing_path[i].velocity = (smoothing_path[i].distance - smoothing_path[i -
                                      1].distance) / (smoothing_path[i].time - smoothing_path[i - 1].time);
    }
    return smoothing_path;
}

double getAngle(const Node& node, const Node& parent_node,
                const Spline& curve_x, const Spline& curve_y) {
    double node_x = curve_x(node.distance);
    double node_y = curve_y(node.distance);
    double parent_x = curve_x(parent_node.distance);
    double parent_y = curve_y(parent_node.distance);
    double angle = atan2(node_x - parent_x, node_y - parent_y);
    return angle;
}

void RRT::SendVisualization(const std::deque<Node>& smoothing_path,
                            const Spline& curve_x,
                            const Spline& curve_y) {
    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
    out_file_ << "smoothing_path\n";
    for (int i = 0; i < smoothing_path.size(); i++) {
        out_file_ << smoothing_path[i].time << "\t" << smoothing_path[i].distance <<
                  "\t" << smoothing_path[i].velocity << "\n";
    }
    out_file_ << "end_path\n";

    out_file_ << "moving_vehicle\n";
    for (int i = 0; i < 5; i++) {
        int k = i * 10;
        double veh_x = curve_x(smoothing_path[k].distance);
        double veh_y = curve_y_(smoothing_path[k].distance);
        double angle = getAngle(smoothing_path[k + 1], smoothing_path[k], curve_x,
                                curve_y);
        out_file_ << veh_x << "\t" << veh_y << "\t" << angle << "\n";
    }
    out_file_ << "end_vehicle\n";
    out_file_.close();
}
