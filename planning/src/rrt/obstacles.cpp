#include "obstacles.h"
using namespace std;
using namespace planning;

Obstacles::Obstacles() {
    ros::param::get("~rrt/k_risk", k_risk_);
    ros::param::get("~rrt/danger_distance", danger_distance_);
    ros::param::get("~rrt/safe_distance", safe_distance_);
    ros::param::get("~rrt/t_max", t_max_);
    ros::param::get("~rrt/s_max", s_max_);
    ros::param::get("~planning_path", planning_path_);
}

void Obstacles::SetObstacles(const planning::ObstacleMap& obstacle_map) {
    obstacles_ = obstacle_map.dynamic_obstacles;
}

bool Obstacles::CollisionFree(const Node& parent_node, const Node& child_node,
                              const Spline& curve_x, const Spline& curve_y) {
    for (int i = 0; i < obstacles_.size(); i++) {
        planning::DynamicObstacle obs = obstacles_[i];
        double t = parent_node.time;
        while (t <= child_node.time) {
            double obs_pos_x = obs.x + obs.velocity * t * sin(obs.theta);
            double obs_pos_y = obs.y + obs.velocity * t * cos(obs.theta);
            double vel = (parent_node.distance - child_node.distance) /
                         (parent_node.time - child_node.time);
            double s = parent_node.distance + vel * (t - parent_node.time);
            double vehicle_x = curve_x(s);
            double vehicle_y = curve_y(s);
            double dist = sqrt(pow(obs_pos_x - vehicle_x, 2) + pow(obs_pos_y - vehicle_y,
                               2));
            t = t + 0.1;
            if (dist < danger_distance_) {
                return false;
            }
        }
    }
    return true;
}

double Obstacles::NonlinearRisk(double input) {
    if (input > safe_distance_) return 0;

    return k_risk_ / (input - danger_distance_);
}

double Obstacles::RiskAssessment(const std::deque<Node>& path,
                                 const Spline& curve_x, const Spline& curve_y) {
    if (obstacles_.empty()) {
        return 0;
    }
    std::vector<double> min_dis;
    for (int i = 0; i < obstacles_.size(); i++) {
        planning::DynamicObstacle obs = obstacles_[i];
        std::vector<double> dis;
        for (int i = 0; i < path.size(); i++) {
            double vehicle_x = curve_x(path[i].distance);
            double vehicle_y = curve_y(path[i].distance);
            double obs_pos_x = obs.x + obs.velocity * path[i].time * sin(obs.theta);
            double obs_pos_y = obs.y + obs.velocity * path[i].time * cos(obs.theta);
            double ss = sqrt(pow(obs_pos_x - vehicle_x, 2) + pow(obs_pos_y - vehicle_y,
                             2));
            // cout << "ss:" << ss << endl;
            dis.push_back(ss);
        }
        min_dis.push_back(*std::min_element(dis.begin(), dis.end()));
        // cout << "min_dis:" << *std::min_element(dis.begin(), dis.end()) << endl;
    }
    double min_min_dis = *std::min_element(min_dis.begin(), min_dis.end());
    // cout << "min_min_dis:" << min_min_dis << endl;

    double risk = NonlinearRisk(min_min_dis);
    // cout << "risk:" << risk << endl;

    return risk;
}

void Obstacles::InitializeDistanceMap(
    const planning::Pose vehicle_state,
    const Spline& curve_x,
    const Spline& curve_y,
    double s0) {
    init_vehicle_path_length_ = s0;
    int nt = static_cast<int>(t_max_ / kDeltaT) + 1;
    int ns = static_cast<int>(s_max_ / kDeltaS) + 1;
    // Init distance map.
    distance_map_.clear();
    for (int i = 0; i <= nt; i++) {
        std::vector<double> dd;
        for (int j = 0; j <= ns; j++) {
            dd.push_back(0);
        }
        distance_map_.push_back(dd);
    }

    if (obstacles_.size() == 0) {
        return;
    }

    for (int i = 0; i <= nt; i++) {
        for (int j = 0; j <= ns; j++) {
            double t = i * kDeltaT;
            double s = j * kDeltaS;
            double vehicle_x = curve_x(s + s0);
            double vehicle_y = curve_y(s + s0);
            double ss = 10000;
            for (int k = 0; k < obstacles_.size(); k++) {
                planning::DynamicObstacle obs = obstacles_[k];
                double obs_pos_x = obs.x + obs.velocity * t * sin(obs.theta);
                double obs_pos_y = obs.y + obs.velocity * t * cos(obs.theta);
                double dis = sqrt(pow(obs_pos_x - vehicle_x, 2) + pow(obs_pos_y - vehicle_y,
                                  2));
                if (ss > dis) ss = dis;
                // cout << "t:" << t << ", s:" << s << ", obs_x:" << obs_pos_x << "obs_y:" << obs_pos_y
                 //   << "veh:" << vehicle_x << "," << vehicle_y << endl;
            }
            distance_map_[i][j] = ss;
        }
    }
    recordDistanceMap();
    return;
}

bool Obstacles::DistanceCheck(const Node& node) {
    int index_t = static_cast<int>(node.time / kDeltaT);
    int index_s = static_cast<int>((node.distance - init_vehicle_path_length_) /
                                   kDeltaS);
    double dist = distance_map_[index_t][index_s];

    if (dist < danger_distance_) {
        return false;
    } else {
        return true;
    }
}

void Obstacles::recordDistanceMap() {
    int nt = static_cast<int>(t_max_ / kDeltaT) + 1;
    int ns = static_cast<int>(s_max_ / kDeltaS) + 1;
    std::string file_name_ = planning_path_ + "/log/distance_map.txt";
    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
    out_file_ << init_vehicle_path_length_ << "\t";
    for(int j = 1; j <=ns; j++){
        out_file_ << 0 << "\t";
    }
    out_file_ << "\n";

    for (int i = 0; i <= nt; i++) {
        for (int j = 0; j <= ns; j++) {
            double t = i * kDeltaT;
            double s = j * kDeltaS;
            double dist = distance_map_[i][j];
            out_file_ << dist << "\t";
        }
        out_file_ << "\n";
    }
    out_file_.close();
}
