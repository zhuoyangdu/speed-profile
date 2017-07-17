#include "planning_node.h"

namespace planning{

PlanningNode::PlanningNode(const ros::NodeHandle& nh){

    ros::param::get("~single_test", single_test_);

    GetGeometryPath();

    rrt_ptr_ = std::move(std::unique_ptr<RRT> (new RRT));
}

void PlanningNode::Start(){
    pub_trajectory_ =
        nh_.advertise<planning::Trajectory>("/planning/trajectory", rate_);
    sub_vehicle_state_ =
        nh_.subscribe("/simulation/localize", vehicle_state_queue_size_,
        &PlanningNode::VehicleStateCallback, this);
    sub_obstacle_ =
        nh_.subscribe("/simulation/obstacles", obstacles_queue_size_,
        &PlanningNode::ObstacleCallback, this);
    ros::Rate loop_rate(rate_);

    if(!single_test_){
        while(ros::ok()) {
            ros::spinOnce();
            cout << "localize_ready:" << localize_ready_ << ", obstacle_ready:" << obstacle_ready_ << endl;
            if(localize_ready_ && obstacle_ready_) {
                planning::Trajectory trajectory;
                rrt_ptr_->GenerateTrajectory(vehicle_state_, obstacle_map_,
                                             curve_x_, curve_y_, &trajectory);
                pub_trajectory_.publish(trajectory);
            }
            loop_rate.sleep();
        }
    } else{
        vehicle_state_.timestamp = 54000.4;
        vehicle_state_.x = 502.55;
        vehicle_state_.y = 481.2;
        vehicle_state_.theta = 0;
        vehicle_state_.length = 0;
        vehicle_state_.velocity = 4;

        DynamicObstacle obs;
        obs.timestamp = 54000.4;
        obs.id = "veh2";
        obs.x = 519.9628;
        obs.y = 502.55;
        obs.theta = 270.0/180* M_PI;
        obs.velocity = 8;
        std::vector<DynamicObstacle> dynamic_obstacles;
        dynamic_obstacles.push_back(obs);
        obstacle_map_.dynamic_obstacles = dynamic_obstacles;
        planning::Trajectory trajectory;
        rrt_ptr_->GenerateTrajectory(vehicle_state_, obstacle_map_,
                                     curve_x_, curve_y_, &trajectory);
    }
}

void PlanningNode::VehicleStateCallback(const planning::Pose& localize){
    vehicle_state_ = localize;
    cout << "heard localize" << endl;
    localize_ready_ = true;
}

void PlanningNode::ObstacleCallback(const planning::ObstacleMap& obstacle_map){
    obstacle_map_ = obstacle_map;
    cout << "heard obstacle" << endl;
    obstacle_ready_ = true;
}

void SplitString(const std::string& s, const std::string& c,
                 std::vector<std::string>* v) {
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2) {
        v->push_back(s.substr(pos1, pos2-pos1));
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v->push_back(s.substr(pos1));
}

void PlanningNode::GetGeometryPath(){
    std::vector<double> xs,ys;
    std::string line;
    std::ifstream file("/home/parallels/workspace/catkin_ws/planning/data/RoadXY.txt");
    if(file.is_open()){
        ROS_INFO("reading road config.");
        int i;
        while(getline(file, line)){
            std::vector<std::string> ps;
            SplitString(line, "\t", &ps);
            if(ps.size()<3){
                continue;
            }
            double x,y;
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

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    planning::PlanningNode planning_node(nh);
    planning_node.Start();
    return 0;
}
