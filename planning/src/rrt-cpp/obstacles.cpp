#include "obstacles.h"
using namespace std;
using namespace planning;

Obstacles::Obstacles(){
    ros::param::get("~rrt/k_risk", k_risk_);
    ros::param::get("~rrt/danger_distance", danger_distance_);
    ros::param::get("~rrt/safe_distance", safe_distance_);
}

void Obstacles::SetObstacles(const planning::ObstacleMap& obstacle_map){
    obstacles_ = obstacle_map.dynamic_obstacles;
}

bool Obstacles::CollisionFree(const Node& parent_node, const Node& child_node,
                              const Spline& curve_x, const Spline& curve_y){
    for(int i = 0; i < obstacles_.size(); i++){
        planning::DynamicObstacle obs = obstacles_[i];
        double t = parent_node.time;
        while(t <= child_node.time){
            double obs_pos_x = obs.x + obs.velocity * t * sin(obs.theta);
            double obs_pos_y = obs.y + obs.velocity * t * cos(obs.theta);
            double vel = (parent_node.distance - child_node.distance) / (parent_node.time - child_node.time);
            double s = parent_node.distance + vel * (t - parent_node.time);
            double vehicle_x = curve_x(s);
            double vehicle_y = curve_y(s);
            double dist = sqrt(pow(obs_pos_x-vehicle_x,2)+pow(obs_pos_y-vehicle_y,2));
            t = t + 0.1;
            if(dist < danger_distance_){
                return false;
            }
        }
    }
    return true;
}

double Obstacles::NonlinearRisk(double input){
    if(input > safe_distance_) return 0;

    return k_risk_ / (input - danger_distance_);
}

double Obstacles::RiskAssessment(const std::deque<Node>& path,
                                 const Spline& curve_x, const Spline& curve_y){
    std::vector<double> min_dis;
    for(int i = 0; i < obstacles_.size(); i++){
        planning::DynamicObstacle obs = obstacles_[i];
        std::vector<double> dis;
        for(int i = 0; i < path.size(); i++){
            double vehicle_x = curve_x(path[i].distance);
            double vehicle_y = curve_y(path[i].distance);
            double obs_pos_x = obs.x + obs.velocity * path[i].time * sin(obs.theta);
            double obs_pos_y = obs.y + obs.velocity * path[i].time * cos(obs.theta);
            double ss = sqrt(pow(obs_pos_x-vehicle_x,2)+pow(obs_pos_y-vehicle_y,2));
            cout << "ss:" << ss << endl;
            dis.push_back(ss);
        }
        min_dis.push_back(*std::min_element(dis.begin(), dis.end()));
        cout << "min_dis:" << *std::min_element(dis.begin(), dis.end()) << endl;
    }
    double min_min_dis = *std::min_element(min_dis.begin(), min_dis.end());
    cout << "min_min_dis:" << min_min_dis << endl;

    double risk = NonlinearRisk(min_min_dis);
    cout << "risk:" << risk << endl;

    return risk;
}