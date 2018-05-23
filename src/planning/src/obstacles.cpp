#include "obstacles.h"
using namespace std;
using namespace planning;

Obstacles::Obstacles() {
    ros::param::get("~rrt/k_risk", k_risk_);
    ros::param::get("~rrt/danger_distance", danger_distance_);
    ros::param::get("~rrt/safe_distance", safe_distance_);
    ros::param::get("~rrt/t_max", t_max_);
    ros::param::get("~rrt/s_max", s_max_);
    ros::param::get("~rrt/collision_distance", collision_distance_);
    ros::param::get("~planning_path", planning_path_);
    ros::param::get("~rrt/safe_ttc", safe_ttc_);
    ros::param::get("~rrt/t_goal", t_goal_);
    ros::param::get("~rrt/max_vel", max_vel_);
}

void Obstacles::SetObstacles(const common::ObstacleMap& obstacle_map) {
    obstacles_ = obstacle_map.dynamic_obstacles;
}

bool Obstacles::CollisionFree(const Node& parent_node, const Node& child_node,
                              const Spline& curve_x, const Spline& curve_y) {
    for (int i = 0; i < obstacles_.size(); i++) {
        common::DynamicObstacle obs = obstacles_[i];
        double t = parent_node.time;
        while (t <= child_node.time) {
            double obs_pos_x = obs.x + obs.velocity * t * sin(obs.theta);
            double obs_pos_y = obs.y + obs.velocity * t * cos(obs.theta);
            double vel = (parent_node.distance - child_node.distance) /
                         (parent_node.time - child_node.time);
            double s = parent_node.distance + vel * (t - parent_node.time);
            double vehicle_x = curve_x(s);
            double vehicle_y = curve_y(s);
            double dist = pow(pow(obs_pos_x - vehicle_x, 2) + pow(obs_pos_y - vehicle_y,
                              2), 0.5);
            t = t + 0.5;
            if (dist < collision_distance_) {
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
    double max_ttc = safe_ttc_;
    for (int i = 0; i < path.size(); i++) {
        double node_ttc = ComputeTTC(path[i].time, path[i].distance,
                                     path[i].velocity, curve_x, curve_y);
        if (node_ttc >= max_ttc) {
            continue;
        } else {
            max_ttc = node_ttc;
        }
    }
    double risk;
    if (max_ttc < safe_ttc_) {
        risk = 1 / max_ttc;
    } else {
        risk = 0;
    }

    return risk;
}

void Obstacles::InitializeDistanceMap(
    const common::Pose& vehicle_state,
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
            double ss = safe_distance_;
            for (int k = 0; k < obstacles_.size(); k++) {
                common::DynamicObstacle obs = obstacles_[k];
                double obs_pos_x = obs.x + obs.velocity * t * sin(obs.theta);
                double obs_pos_y = obs.y + obs.velocity * t * cos(obs.theta);
                double dis = pow(pow(obs_pos_x - vehicle_x, 2) + pow(obs_pos_y - vehicle_y,
                                 2), 0.5);
                if (ss > dis) ss = dis;
            }
            distance_map_[i][j] = ss;
        }
    }
    recordDistanceMap();
    std::cout << "record distance map" << recordDistanceMapProto() << std::endl;
    return;
}

double Obstacles::EuclideanDisToObs(double x, double y, double t) {
    double dis = 1000;
    for (int k = 0; k < obstacles_.size(); k++) {
        double obs_x = obstacles_[k].x + obstacles_[k].velocity * t * sin(
                           obstacles_[k].theta);
        double obs_y = obstacles_[k].y + obstacles_[k].velocity * t * cos(
                           obstacles_[k].theta);
        double od = pow(pow(x - obs_x, 2) + pow(y - obs_y,
                                                2), 0.5);
        if (dis > od) dis = od;
    }
    return dis;
}

double Obstacles::ComputeTTC(double node_time, double node_distance,
                             double node_vel,
                             const Spline& curve_x, const Spline& curve_y) {

    double ttc = 10;
    if (obstacles_.size() == 0) {
        return ttc;
    }

    for (int tt = 0; tt < 5; tt++) {
        double t = tt * 0.1;
        double veh_x = curve_x(node_distance + t * node_vel);
        double veh_y = curve_y(node_distance + t * node_vel);
        double distance = EuclideanDisToObs(veh_x, veh_y, t + node_time);
        if (distance < collision_distance_) {
            ttc = t;
            break;
        }
    }
    return ttc;
}

std::vector<std::vector<double>> Obstacles::ComputeTTCForFixedVel(
                                  double current_path_length,
                                  double node_vel,
                                  const Spline& curve_x,
const Spline& curve_y) {
    std::string file_name_ = planning_path_ + "/log/ttc_map.txt";
    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
    out_file_ << "velocity\t" << node_vel << "\n";

    std::vector<std::vector<double>> ttc_for_vel;
    //t s v
    for (int tt = 0; tt < 10 * t_max_; tt++) {
        double t = tt * 0.1;
        std::vector<double> ttc_t;
        for (int s = 0; s < s_max_; s++) {
            // Compute ttc of a node. (t, s, v)
            double ttc = ComputeTTC(t, current_path_length + s, node_vel,
                                    curve_x, curve_y);
            ttc_t.push_back(ttc);
            out_file_ << ttc << "\t";
        }
        ttc_for_vel.push_back(ttc_t);
        out_file_ << "\n";
    }
    // out_file_ << "end_vel\n";
    out_file_.close();
    return ttc_for_vel;
}

void Obstacles::ComputeTTCMap(double current_path_length,
                              const Spline& curve_x,
                              const Spline& curve_y) {
    for (int i = 0; i < 20; i++) {
        std::vector<std::vector<double> > ttc_vel;;
        ttc_vel = ComputeTTCForFixedVel(current_path_length, i,
                                        curve_x, curve_y);
    }
}

double Obstacles::ReadDistanceMap(const Node& node) {
    if (obstacles_.size() < 1) {
        return safe_distance_;
    }
    int index_t = static_cast<int>(node.time / kDeltaT);
    int index_s = static_cast<int>((node.distance - init_vehicle_path_length_) /
                                   kDeltaS);
    double dist = distance_map_[index_t][index_s];

    return dist;
}

bool Obstacles::recordDistanceMapProto() {
    std::string file_name = planning_path_ + "/log/distance_map_dbg.pb.txt";
    planning::ObstacleMapDebug obs_debug;
    obs_debug.set_init_path_length(init_vehicle_path_length_);
    obs_debug.set_delta_t(kDeltaT);
    obs_debug.set_delta_s(kDeltaS);
    obs_debug.set_t_goal(t_goal_);
    obs_debug.set_max_vel(max_vel_);
    obs_debug.set_danger_distance(danger_distance_);
    return common::SetProtoToASCIIFile(obs_debug, file_name);
}

void Obstacles::recordDistanceMap() {
    int nt = static_cast<int>(t_max_ / kDeltaT) + 1;
    int ns = static_cast<int>(s_max_ / kDeltaS) + 1;
    std::string file_name_ = planning_path_ + "/log/distance_map.txt";
    std::ofstream out_file_(file_name_.c_str(), std::ios::in | std::ios::app);
    out_file_ << init_vehicle_path_length_ << "\t";
    for (int j = 1; j <= ns; j++) {
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
