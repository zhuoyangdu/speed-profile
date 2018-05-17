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
            // vehicle_state_ = single_test_vehicle_;
            common::Trajectory trajectory;
            rrt_ptr_->GenerateTrajectory(vehicle_state_, obstacle_map_,
                                          &route_, &trajectory);
            ros::spin();
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
    if(single_test_) {

        std::string file_name = planning_path_ + "/config/junction_test_config.pb.txt";
        if(!common::GetProtoFromASCIIFile(file_name, &env_conf_)) {
            ROS_ERROR("Error read config!");
        }

        // Read params.
        vehicle_state_.timestamp = env_conf_.init_vehicle_state().timestamp0();
        vehicle_state_.x = env_conf_.init_vehicle_state().x0();
        vehicle_state_.y = env_conf_.init_vehicle_state().y0();
        vehicle_state_.theta = env_conf_.init_vehicle_state().theta0();
        vehicle_state_.velocity = env_conf_.init_vehicle_state().v0();
        vehicle_state_.acceleration = env_conf_.init_vehicle_state().a0();

        std::vector<common::DynamicObstacle> dynamic_obstacles;
        for (int i = 0; i < env_conf_.obstacle_size(); ++i) {
            common::DynamicObstacle obs;
            obs.id = env_conf_.obstacle(i).id();
            obs.x = env_conf_.obstacle(i).x();
            obs.y = env_conf_.obstacle(i).y();
            obs.theta = env_conf_.obstacle(i).theta();
            obs.velocity = env_conf_.obstacle(i).v();
            obs.acceleration = env_conf_.obstacle(i).a();
        }
        obstacle_map_.dynamic_obstacles = dynamic_obstacles;
    }
}

}  // namespace planning

int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    planning::PlanningNode planning_node(nh);
    planning_node.Start();
    return 0;
}
