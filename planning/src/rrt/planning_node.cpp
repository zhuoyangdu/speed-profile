#include "planning_node.h"
using namespace std;

namespace planning {

PlanningNode::PlanningNode(const ros::NodeHandle& nh) {

    ParamConfig();

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
             if (!trajectory.poses.empty()){
                    pub_trajectory_.publish(trajectory);
            }
            }
            loop_rate.sleep();
        }
    } else {
        vehicle_state_ = single_test_vehicle_;
        obstacle_map_.dynamic_obstacles = single_test_obstacles_;

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
    std::string file_name = planning_path_ + "/data/path/" + road_file_;
    cout << "file name" << file_name << endl;
    std::ifstream file(file_name);
    if (file.is_open()) {
        ROS_INFO("Reading road config.");
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
            cout << "x:" << x << ", y:" << y << endl;
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

void PlanningNode::ParamConfig() {
    ros::param::get("~single", single_test_);
    ros::param::get("~planning_path", planning_path_);
    ros::param::get("~road_file", road_file_);
    if (single_test_) {
        std::string single_test_case;
        int collision_number;
        ros::param::get("~single_test/single_test_case", single_test_case);
        ros::param::get("~single_test/collision_number", collision_number);

        // Read init vehicle state.
        single_test_vehicle_.timestamp = 54000;
        ros::param::get("~" + single_test_case + "/x0", single_test_vehicle_.x);
        ros::param::get("~" + single_test_case + "/y0", single_test_vehicle_.y);
        ros::param::get("~" + single_test_case + "/theta0",
                        single_test_vehicle_.theta);
        ros::param::get("~" + single_test_case + "/v0",
                        single_test_vehicle_.velocity);
        ros::param::get("~" + single_test_case + "/a0",
                        single_test_vehicle_.acceleration);
        ros::param::get("~" + single_test_case + "/road_file", road_file_);

        // Read dynamic obstacles.
        std::vector<DynamicObstacle> dynamic_obstacles;
        if (collision_number == 0) {
            std::cout << "No obstacle in single test." << std::endl;
        } else if (collision_number == 1 || collision_number == 2) {
            DynamicObstacle obs;
            obs.id = "obs1";
            ros::param::get("~" + single_test_case + "/obs1_x0", obs.x);
            ros::param::get("~" + single_test_case + "/obs1_y0", obs.y);
            ros::param::get("~" + single_test_case + "/obs1_theta0", obs.theta);
            ros::param::get("~" + single_test_case + "/obs1_v0", obs.velocity);
            dynamic_obstacles.push_back(obs);
            if (collision_number == 2) {
                DynamicObstacle obs;
                obs.id = "obs2";
                ros::param::get("~" + single_test_case + "/obs2_x0", obs.x);
                ros::param::get("~" + single_test_case + "/obs2_y0", obs.y);
                ros::param::get("~" + single_test_case + "/obs2_theta0", obs.theta);
                ros::param::get("~" + single_test_case + "/obs2_v0", obs.velocity);
                dynamic_obstacles.push_back(obs);
            }
        } else {
            ROS_ERROR("The number of obstacle is out of range.");
        }
        // }
        single_test_obstacles_ = dynamic_obstacles;
        std::cout << "Single test case:" << single_test_case << std::endl;
        std::cout << "Initial state:" << std::endl;
        std::cout << "  vehicle state: " << single_test_vehicle_.x << ", " <<
                  single_test_vehicle_.y << ", " << single_test_vehicle_.theta << ", " <<
                  single_test_vehicle_.velocity << std::endl;
        std::cout << "Obstacles: " <<  collision_number << " in total" << std::endl;
        for (int i = 0; i < collision_number; i++) {
            std::cout << "   obstacle " << i + 1 << ": " << dynamic_obstacles[i].x << ", "
                      << dynamic_obstacles[i].y << ", " << dynamic_obstacles[i].theta <<
                      ", " << dynamic_obstacles[i].velocity << std::endl;
        }

    }
}

} // namespace planning

int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    planning::PlanningNode planning_node(nh);
    planning_node.Start();
    return 0;
}
