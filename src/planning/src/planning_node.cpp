#include <string>
#include "planning_node.h"
using namespace std;

namespace planning {

PlanningNode::PlanningNode(const ros::NodeHandle& nh) {
    // Read the planning parameters and environment settings.
    ParamConfig();

    // Initialize the rrt planner.
    rrt_ptr_ = std::move(std::unique_ptr<RRT> (new RRT));
}

void PlanningNode::Start() {
    // For replanning.
    // Publish trajectory for the vehicle.
    pub_trajectory_ =
        nh_.advertise<common::Trajectory>("/planning/trajectory", rate_);

    // Subscribe vehicle state and traffic information.
    sub_vehicle_state_ =
        nh_.subscribe("/simulation/localize", vehicle_state_queue_size_,
                      &PlanningNode::VehicleStateCallback, this);
    sub_obstacle_ =
        nh_.subscribe("/simulation/obstacles", obstacles_queue_size_,
                      &PlanningNode::ObstacleCallback, this);

    // Set the replanning period.
    ros::Rate loop_rate(rate_);

    // There are two types of simulation: single test for debug and replanning test
    // for real-time traffic. The information of single test is given by the
    // config file, and the replanning test subscribes information from the
    // simulation environment SUMO.
    if (!single_test_) {
        while (ros::ok()) {
            ros::spinOnce();
            // Wait for localize message and traffic condition message.
            if (localize_ready_ && obstacle_ready_) {
                common::Trajectory trajectory;
                rrt_ptr_->GenerateTrajectory(vehicle_state_, obstacle_map_,
                                             &route_, &trajectory);
                pub_trajectory_.publish(trajectory);
            }
            loop_rate.sleep();
        }
    } else {
            planning_vis_.PubEnv();
            // The single test mode only generates a trajectory for one period.
            vehicle_state_ = single_test_vehicle_;
            obstacle_map_.dynamic_obstacles = single_test_obstacles_;
            common::Trajectory trajectory;
            //rrt_ptr_->GenerateTrajectory(vehicle_state_, obstacle_map_,
            //                              &route_, &trajectory);
            return;
    }
}

void PlanningNode::VehicleStateCallback(const common::Pose& localize) {
    vehicle_state_ = localize;
    localize_ready_ = true;
}

void PlanningNode::ObstacleCallback(const common::ObstacleMap&
                                    obstacle_map) {
    obstacle_map_ = obstacle_map;
    obstacle_ready_ = true;
}

void PlanningNode::ParamConfig() {
    ros::param::get("~single", single_test_);
    ros::param::get("~planning_path", planning_path_);
    ros::param::get("~road_file", road_file_);
    /*
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
        std::vector<common::DynamicObstacle> dynamic_obstacles;
        if (collision_number == 0) {
            std::cout << "No obstacle in single test." << std::endl;
        } else if (collision_number == 1 || collision_number == 2) {
            common::DynamicObstacle obs;
            obs.id = "obs1";
            ros::param::get("~" + single_test_case + "/obs1_x0", obs.x);
            ros::param::get("~" + single_test_case + "/obs1_y0", obs.y);
            ros::param::get("~" + single_test_case + "/obs1_theta0", obs.theta);
            ros::param::get("~" + single_test_case + "/obs1_v0", obs.velocity);
            dynamic_obstacles.push_back(obs);
            if (collision_number == 2) {
                common::DynamicObstacle obs;
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
        single_test_obstacles_ = dynamic_obstacles;
        std::cout << "--------------------------------" << std::endl;
        std::cout << "Single test case:" << single_test_case << std::endl;
        std::cout << "Initial state:" << std::endl;
        std::cout << "  vehicle state: " << single_test_vehicle_.x << ", " <<
                  single_test_vehicle_.y << ", " << single_test_vehicle_.theta << ", " <<
                  single_test_vehicle_.velocity << std::endl;
        std::cout << "Obstacles: " << std::endl;
        for (int i = 0; i < collision_number; i++) {
            std::cout << "   obstacle " << i + 1 << ": " << dynamic_obstacles[i].x << ", "
                      << dynamic_obstacles[i].y << ", " << dynamic_obstacles[i].theta <<
                      ", " << dynamic_obstacles[i].velocity << std::endl;
        }
        std::cout << "--------------------------------" << std::endl;
    }
*/

    std::string file_name = planning_path_ + "/config/obstacle_config.pb.txt";
    if(!common::GetProtoFromASCIIFile(file_name, &obstacle_conf_)) {
        ROS_ERROR("Error read config!");
    } else {
        std::cout << "obstacle size:" << obstacle_conf_.obstacle_size() << std::endl;
        for (int i = 0; i < obstacle_conf_.obstacle_size(); ++i) {
            std::cout << "obstacle id:" << obstacle_conf_.obstacle(i).id() << "," <<
                obstacle_conf_.obstacle(i).x() << "," << obstacle_conf_.obstacle(i).y() << std::endl;
        }
    }
}

}  // namespace planning

int main(int argc, char** argv) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    planning::PlanningNode planning_node(nh);
    planning_node.Start();
    return 0;
}
