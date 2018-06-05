#include "rrt.h"
#include <sys/time.h>
#include <vector>
#include <string>

using namespace std;
using namespace planning;
RRT::RRT() {
    ros::param::get("~planning_path", planning_path_);
    std::string file_name = planning_path_ + "/config/planning_config.pb.txt";
    if(!common::GetProtoFromASCIIFile(file_name, &planning_conf_)) {
        ROS_ERROR("Error read config!");
    } else {
        rrt_conf_ = planning_conf_.rrt();
    }
}

void RRT::GenerateTrajectory(const common::Pose& vehicle_state,
                             const common::ObstacleMap& obstacle_map,
                             Route* route,
                             common::Trajectory* trajectory) {
    cout << "Planning time:" << vehicle_state.timestamp << endl;
    ROS_INFO("Vehicle state: x: %.3f, y: %.3f, theta: %.3f, v: %.3f",
            vehicle_state.x, vehicle_state.y, vehicle_state.theta,
            vehicle_state.velocity);
    cout << "obstacle size:" << obstacle_map.dynamic_obstacles.size() << endl;
    for (int i = 0; i < obstacle_map.dynamic_obstacles.size(); i++) {
        common::DynamicObstacle obs = obstacle_map.dynamic_obstacles[i];
        ROS_INFO("Obstacle: x: %.3f, y: %.3f, theta: %.3f, v: %.3f",
            obs.x, obs.y, obs.theta, obs.velocity);
    }

    // record.
    newFile();
    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
    for (int i = 0; i < obstacle_map.dynamic_obstacles.size(); i++) {
        common::DynamicObstacle obs = obstacle_map.dynamic_obstacles[i];
        out_file_ << "obstacle\t" << obs.timestamp << "\t" <<
                  obs.id << "\t" << obs.x << "\t" << obs.y << "\t"
                  << obs.theta << "\t" << obs.velocity << "\n";
    }
    out_file_ << "vehicle_state\t" << vehicle_state.timestamp << "\t"
              << vehicle_state.x << "\t" << vehicle_state.y << "\t"
              << vehicle_state.theta << "\t" << vehicle_state.velocity << "\n";

    // Initialize path.
    route_ = route_;
    curve_x_ = route->get_x();
    curve_y_ = route->get_y();
    double s0 = GetGeometryPathLength(vehicle_state.x, vehicle_state.y);
    cout << "s0:" << s0 << endl;

    // Initialize obstacles.
    obstacles.SetObstacles(obstacle_map);
    obstacles.InitializeDistanceMap(vehicle_state, curve_x_, curve_y_, s0);

    // Initialize tree.
    double length = 0;
    Node first_node(0, s0);
    first_node.velocity = vehicle_state.velocity;
    first_node.acceleration = vehicle_state.acceleration;
    first_node.self_id = 0;
    first_node.parent_id = -1;
    tree_ = {first_node};

    PrintTree();

    int n_sample = 0;
    int n_feasible = 0;
    int n_path = 0;
    double min_cost = 10000;
    std::deque<Node> min_path;

    clock_t start, ends;
    start = clock();
    srand(time(NULL));
    while (n_sample < rrt_conf_.max_failed_attemptes()) {

        // Sample.
        Node sample = RandomSample(s0);

        if (obstacles.ReadDistanceMap(sample) < rrt_conf_.collision_distance()) {
            continue;
        }

        n_sample = n_sample + 1;

        // Extend.
        bool node_valid;
        Node new_node(-1, -1, -1);
        Extend(sample, &new_node, &node_valid);
        // cout << "new_node:" << new_node.time << "," << new_node.distance << ",feasible" << node_valid << std::endl;

        if (node_valid) {
            n_feasible = n_feasible + 1;
            if (ReachingGoal(new_node)) {
                n_path = n_path + 1;
                std::deque<Node> path = GetParentPath(new_node);
                std::vector<double> path_cost = GetPathCost(path);
                cout << "risk: " << rrt_conf_.kr() * path_cost[0]
                    << ", smoothness: " << rrt_conf_.ks() * path_cost[1]
                    << ", e_val: " << rrt_conf_.kv() * path_cost[2] << std::endl;
                double cost_sum = WeightingCost(path_cost);
                if (cost_sum < min_cost) {
                    min_path = path;
                    min_cost = cost_sum;
                }
            }
        }
    }

    cout << "min_cost:" << std::endl;
    std::vector<double> path_cost = GetPathCost(min_path);
    cout << "risk: " << rrt_conf_.kr() * path_cost[0]
        << ", smoothness: " << rrt_conf_.ks() * path_cost[1]
        << ", e_val: " << rrt_conf_.kv() * path_cost[2] << std::endl;

    cout << endl;
    cout << "------Result------" << endl;
    cout << "Total attemps:" << n_sample << ", feasible sample:" << n_feasible <<
         endl;
    ends = clock();
    cout << "Elapsed time:" << double(ends - start) / CLOCKS_PER_SEC << endl;

    if (n_path > 0) {
        std::deque<Node> final_path = PostProcessing(min_path);
        SendVisualization(final_path, curve_x_, curve_y_);
        std::vector<common::Pose> poses;
        // std::cout << "final trajectory:" << std::endl;
        for (int i = 0; i < min_path.size(); i++) {
            common::Pose pose;
            pose.timestamp = min_path[i].time + vehicle_state.timestamp;
            pose.velocity = min_path[i].velocity;
            pose.length = min_path[i].distance;
            pose.x = curve_x_(pose.length);
            pose.y = curve_y_(pose.length);
            poses.push_back(pose);
            // std::cout << "x:" << pose.x << ", y:" << pose.y << ", theta:" << pose.theta << std::endl;
        }
        trajectory->poses = poses;
    } else {
        ROS_ERROR("No path found.");
    }

    cout << endl;
    // PrintTree();
    return;
}


void RRT::Extend(Node& sample, Node* new_node, bool* node_valid) {

    // Find nearest node.
    Node nearest_node;
    GetNearestNode(sample, &nearest_node, node_valid);
    // cout << "nearest node:" << nearest_node.time << "," << nearest_node.distance << std::endl;

    if (!*node_valid) {
        return;
    }

    // Steer to get new node.
    Steer(sample, nearest_node, new_node);
    if (new_node->time > rrt_conf_.t_goal()) {
        *node_valid = false;
        return;
    }

    bool vertex_feasible = VertexFeasible(nearest_node, *new_node);

    if (vertex_feasible) {
        // cout << "A feasible vertex!!!!!!!!!!!!!!!!" << endl;
        *node_valid = true;
        ChooseParent(nearest_node, new_node);
        Rewire(*new_node);
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
        if (vertex_feasible) {
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
                for (int k = 0; k < near_node.children_id.size(); k++) {
                    int child_index = near_node.children_id[k];
                    tree_[child_index].acceleration = ComputeAcceleration(near_node,
                                                      tree_[child_index]);
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
    tree_.push_back(*new_node);
    tree_[min_node.self_id].children_id.push_back(new_node->self_id);
    if (new_node->time > max_tree_t_) {
        max_tree_t_ = new_node->time;
    }
}

double RRT::WeightingCost(std::vector<double>& cost) {
    double w = rrt_conf_.kr() * cost[0] + rrt_conf_.ks() * cost[1]
                + rrt_conf_.kv() * cost[2];
    return w;
}

Node RRT::RandomSample(double s0) {
    double sample_t = 0;
    double sample_s = 0;
    double sample_s_range = rrt_conf_.s_max() < rrt_conf_.t_max() * rrt_conf_.max_vel()
                            ? rrt_conf_.s_max() : rrt_conf_.t_max() * rrt_conf_.max_vel();
    sample_t = (double) rand() / RAND_MAX * rrt_conf_.t_max();
    double s_range = sample_t * rrt_conf_.max_vel();;
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
    double min_dist = 10000;
    double min_index = -1;

    for (int i = 0; i < tree_.size(); i++) {
        double delta_s = sample.distance - tree_[i].distance;
        double delta_t = sample.time - tree_[i].time;

        if (delta_s < -0.5 || delta_t < 0) {
            continue;
        } else {
            double vel = delta_s / delta_t;
            if (vel > rrt_conf_.max_vel()) {
                continue;
            }
            if (delta_s <= 0) {
                vel = 0;
            }
            double acc = (tree_[i].velocity - vel) / delta_t;
            if (fabs(acc) > rrt_conf_.max_acc()) {
                continue;
            } else {
                double dist = 2*fabs(acc) + delta_t;
                if (dist < min_dist) {
                    min_dist = dist;
                    min_index = i;
                }
            }
        }
    }
    if (min_index == -1) {
        *node_valid = false;
        return;
    } else {
        *node_valid = true;
        *nearest_node = tree_[min_index];
        return;
    }
}

void RRT::Steer(const Node& sample, const Node& nearest_node,
                Node* new_node) {

    double k = ComputeVelocity(sample, nearest_node);
    if (k <= 0) k = 0;
    new_node->time = nearest_node.time + rrt_conf_.dt();
    new_node->distance = nearest_node.distance + k * rrt_conf_.dt();
    new_node->velocity = k;
    new_node->self_id = tree_.size();
    return;
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
    if (vel > rrt_conf_.max_vel()) {
        // ROS_ERROR("[unfeasible] velocity: %.3f, max_vel: %.3f", vel, rrt_conf_.max_vel());
        return false;
    }

    double acc = ComputeAcceleration(parent_node, child_node);
    if (fabs(acc) > rrt_conf_.max_acc()) {
        // ROS_ERROR("[unfeasible] acceleration: %.3f, max_acc: %.3f", acc, rrt_conf_.max_acc());
        return false;
    }

    double curvature = route_.GetCurvature(parent_node.distance/2 + child_node.distance/2);
    double heading_rate = vel * curvature;
    if (fabs(heading_rate) > rrt_conf_.max_heading_rate()) {
        ROS_ERROR("heading rate too large");
        return false;
    }

    if (parent_node.self_id == 0) {
        if (abs(acc - parent_node.acceleration) > 2.5) {
            // ROS_ERROR("[unfeasible] jerk");
            return false;
        }
    }

    bool collision_free = obstacles.CollisionFree(parent_node, child_node,
                          curve_x_, curve_y_);
    if (!collision_free) {
        // ROS_ERROR("[unfeasible] collision");
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
        double acc = path[i].acceleration;
        vector_acc.push_back(acc);
        sum_acc = sum_acc + acc;
        sum_abs_acc = sum_abs_acc + fabs(acc);
    }

    double average_acc = sum_acc / vector_acc.size();
    double variance_acc = 0;
    for (int i = 0; i < vector_acc.size(); i++) {
        variance_acc = variance_acc + abs(vector_acc[i] - average_acc);
    }
    variance_acc = abs(variance_acc) / vector_acc.size() * 10;
    variance_acc = abs(vector_acc[1] - vector_acc[0]) + variance_acc;

    double sum_jerk = 0;
    for (int i = 0; i < vector_acc.size()-1; ++i) {
        sum_jerk += fabs(vector_acc[i+1] - vector_acc[i]);
    }

    return variance_acc + rrt_conf_.k_jerk() * sum_jerk;
}

double RRT::GetPathVelError(const std::deque<Node>& path) {
    double ev = 0;
    for (int i = 0; i < path.size(); i++) {
        ev = ev + fabs(path[i].velocity - rrt_conf_.v_goal());
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

std::vector<double> RRT::GetSingleNodeCost(const Node& node) {
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
            if (fabs(delta_acc) < rrt_conf_.lower_range_a() && fabs(vel) < rrt_conf_.max_vel()) {
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
            if (fabs(delta_acc) < rrt_conf_.lower_range_a() && vel < rrt_conf_.max_vel()) {
                near_region.push_back(tree_[i]);
            }
        }
    }
    return near_region;
}

bool RRT::ReachingGoal(const Node& node) {
    if (abs(node.time - rrt_conf_.t_goal()) < 0.3) {
        return true;
    }
    return false;
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
        double a = (path[i+1].velocity - path[i].velocity) / (path[i+1].time - path[i].time);
        for (int k = 0; k < n - 0.5; k++) {
            Node node;
            node.time = path[i].time + k * rrt_conf_.dt();
            node.velocity = path[i].velocity + a *k * rrt_conf_.dt();
            node.distance = path[i].distance
                + (path[i].velocity + node.velocity) * k * rrt_conf_.dt() / 2;
            full_path.push_back(node);
        }
    }
    full_path.push_back(path.back());
    return full_path;
}

void RRT::SendVisualization(const std::deque<Node>& final_path,
                            const Spline& curve_x,
                            const Spline& curve_y) {
    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);

    out_file_ << "final_path\n";
    for (int i = 0; i < final_path.size()-1; i++) {
        double s = final_path[i].distance;
        out_file_ << final_path[i].time << "\t" << s << "\t"
                << final_path[i].velocity << "\t" << route_.x(s)
                << "\t" << route_.y(s) << "\t"
                << route_.theta(s) << "\n";
    }
    out_file_ << "end_path\n";

    out_file_ << "tree\n";
    for (int i = 0; i < tree_.size(); i++) {
        out_file_ << tree_[i].time << "\t" << tree_[i].distance << "\t" <<
                  tree_[i].velocity << "\t" << tree_[i].parent_id << "\n";
    }
    out_file_ << "end_tree\n";

    out_file_ << "moving_vehicle\n";
    for (int i = 0; i < rrt_conf_.t_goal(); i++) {
        int k = i / rrt_conf_.dt();
        double veh_x = curve_x(final_path[k].distance);
        double veh_y = curve_y(final_path[k].distance);
        double angle = getAngle(final_path[k + 1], final_path[k], curve_x,
                                curve_y);
        out_file_ << veh_x << "\t" << veh_y << "\t" << angle << "\n";
    }
    out_file_ << "end_vehicle\n";

    out_file_ << "moving_obstacle\n";
    std::vector<common::DynamicObstacle> all_obs = obstacles.GetObstacles();
    out_file_ << "obs_size" << "\t" << all_obs.size() << "\n";
    for (common::DynamicObstacle obs : all_obs) {
        out_file_ << "start\n";
        for (int i = 0; i < rrt_conf_.t_goal(); i++) {
            double obs_pos_x = obs.x + obs.velocity * i * sin(obs.theta);
            double obs_pos_y = obs.y + obs.velocity * i * cos(obs.theta);
            double obs_angle = obs.theta;
            out_file_ << obs_pos_x << "\t" << obs_pos_y << "\t" << obs_angle << "\n";
        }
        out_file_ << "end\n";
    }

    out_file_ << "end_obstacle\n";

    out_file_.close();
    return;
}

void RRT::PrintTree() {
    std::cout << "-------------------------" << std::endl;
    std::cout << "-----------tree----------" << std::endl;
    for (Node n : tree_) {
        std::cout << "t: " << n.time << ", s: " << n.distance << ", v: " << n.velocity
            << ", a: " << n.acceleration << std::endl;
    }
    std::cout << "---------end-tree--------" << std::endl;
    std::cout << "-------------------------" << std::endl;
}


void RRT::PrintPath(const std::vector<Node>& path) {
    std::cout << "-------------------------" << std::endl;
    std::cout << "-----------path----------" << std::endl;
    for (Node n : path) {
        std::cout << "t: " << n.time << ", s: " << n.distance << ", v: " << n.velocity
            << ", a: " << n.acceleration << std::endl;
    }
    std::cout << "---------end-path--------" << std::endl;
    std::cout << "-------------------------" << std::endl;
}
