#include "planning_node.h"

namespace planning {

PlanningNode::PlanningNode(const ros::NodeHandle& nh) {

    ros::param::get("~single", single_test_);
    ros::param::get("~planning_path", planning_path_);

    ros::param::get("~single_test/x0", vehicle_x0_);
    ros::param::get("~single_test/y0", vehicle_y0_);
    ros::param::get("~single_test/theta0", vehicle_theta0_);
    ros::param::get("~single_test/s0", vehicle_s0_);
    ros::param::get("~single_test/v0", vehicle_v0_);

    ros::param::get("~single_test/obs1_x0", obs1_x0_);
    ros::param::get("~single_test/obs1_y0", obs1_y0_);
    ros::param::get("~single_test/obs1_theta0", obs1_theta0_);
    ros::param::get("~single_test/obs1_v0", obs1_v0_);

    GetGeometryPath();

    rrt_ptr_ = std::move(std::unique_ptr<RRT> (new RRT));
}

void PlanningNode::Start() {
    pub_trajectory_ =
        nh_.advertise<planning::Trajectory>("/planning/trajectory", rate_);
    sub_vehicle_state_ =
        nh_.subscribe("/simulation/localize", vehicle_state_queue_size_,
                      &PlanningNode::VehicleStateCallback, this);
    sub_obstacle_ =
        nh_.subscribe("/simulation/obstacles", obstacles_queue_size_,
                      &PlanningNode::ObstacleCallback, this);
    ros::Rate loop_rate(rate_);

    if (!single_test_) {
        while (ros::ok()) {
            ros::spinOnce();
            if (localize_ready_ && obstacle_ready_) {
                planning::Trajectory trajectory;
                rrt_ptr_->GenerateTrajectory(vehicle_state_, obstacle_map_,
                                             curve_x_, curve_y_, &trajectory);
                pub_trajectory_.publish(trajectory);
            }
            loop_rate.sleep();
        }
    } else {
        vehicle_state_.timestamp = 54000.2;
        vehicle_state_.x = vehicle_x0_;
        vehicle_state_.y = vehicle_y0_;
        vehicle_state_.theta = vehicle_theta0_;
        vehicle_state_.length = vehicle_s0_;
        vehicle_state_.velocity = vehicle_v0_;

        DynamicObstacle obs1;
        obs1.timestamp = 54000.2;
        obs1.id = "veh1";
        obs1.x = obs1_x0_;
        obs1.y = obs1_y0_;
        obs1.theta = obs1_theta0_;
        obs1.velocity = obs1_v0_;
        DynamicObstacle obs2;
        obs2.timestamp = 54000.2;
        obs2.id = "veh2";
        obs2.x = obs2_x0_;
        obs2.y = obs2_y0_;
        obs2.theta = obs2_theta0_;
        obs2.velocity = obs2_v0_;

        std::vector<DynamicObstacle> dynamic_obstacles;
        dynamic_obstacles.push_back(obs1);
        dynamic_obstacles.push_back(obs2);
        obstacle_map_.dynamic_obstacles = dynamic_obstacles;
        planning::Trajectory trajectory;
        rrt_ptr_->GenerateTrajectory(vehicle_state_, obstacle_map_,
                                     curve_x_, curve_y_, &trajectory);
    }
}

void PlanningNode::VehicleStateCallback(const planning::Pose& localize) {
    vehicle_state_ = localize;
    localize_ready_ = true;
}

void PlanningNode::ObstacleCallback(const planning::ObstacleMap&
                                    obstacle_map) {
    obstacle_map_ = obstacle_map;
    obstacle_ready_ = true;
}

void SplitString(const std::string& s, const std::string& c,
                 std::vector<std::string>* v) {
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2) {
        v->push_back(s.substr(pos1, pos2 - pos1));
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v->push_back(s.substr(pos1));
}

void PlanningNode::GetGeometryPath() {
    std::vector<double> xs, ys;
    std::string line;
    std::string file_name = planning_path_ + "/data/RoadXY.txt";
    std::ifstream file(file_name);
    if (file.is_open()) {
        ROS_INFO("reading road config.");
        int i;
        while (getline(file, line)) {
            std::vector<std::string> ps;
            SplitString(line, "\t", &ps);
            if (ps.size() < 3) {
                continue;
            }
            double x, y;
            x = atof(ps[0].c_str());
            y = atof(ps[1].c_str());
            xs.push_back(x);
            ys.push_back(y);
        }
        file.close();
    } else {
        ROS_ERROR("cannot open path config file.");
    }
    double path_length;
    //Spline::fitCurve();
    Spline::fitCurve(xs, ys, &curve_x_, &curve_y_, &path_length);

}

} // namespace planning

int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    planning::PlanningNode planning_node(nh);
    planning_node.Start();
    return 0;
}
