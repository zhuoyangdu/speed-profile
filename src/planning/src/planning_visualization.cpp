#include "planning_visualization.h"

using namespace std;

namespace planning {
PlanningVisualization::PlanningVisualization() {
    ros::param::get("~planning_path", planning_path_);
    InitPublishers();
}

void PlanningVisualization::InitPublishers() {
    pub_vehicle_state_vis_ =
        nh_.advertise<visualization_msgs::Marker>(
        "/planning_vis/vehicle_state", rate_);
    pub_map_ =
        nh_.advertise<visualization_msgs::Marker>(
        "/planning_vis/map", rate_);
}

void PlanningVisualization::PubEnv() {
    PaintMap();
}

std::vector<std::vector<double>> PlanningVisualization::GetMapFromFile() {
    std::vector<std::vector<double>> map;
    std::string file_name = planning_path_
        + "/../simulation/data/parser/junction_route.txt";
    std::ifstream file(file_name);
    if (file.is_open()) {
        std::string line;
        while(getline(file, line)) {
            common::StringUtils string_utils;
            std::vector<std::string> points;
            string_utils.SplitString(line, " ", &points);
            std::vector<std::string> spoint1, spoint2;
            string_utils.SplitString(points[0], ",", &spoint1);
            string_utils.SplitString(points[1], ",", &spoint2);
            std::vector<double> point12;
            point12.push_back(atof(spoint1[0].c_str()));
            point12.push_back(atof(spoint1[1].c_str()));
            point12.push_back(atof(spoint2[0].c_str()));
            point12.push_back(atof(spoint2[1].c_str()));
            map.push_back(point12);
        }
    } else {
        ROS_ERROR("No such file.");
    }
    return map;
}


void PlanningVisualization::PaintMap() {
    std::vector<std::vector<double>> map = GetMapFromFile();
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/my_frame";
    line_list.id = 1;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 5;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for (std::vector<double> line : map) {
        geometry_msgs::Point p;
        p.x = line[0];
        p.y = line[1];
        line_list.points.push_back(p);
        p.x = line[2];
        p.y = line[3];
        line_list.points.push_back(p);
    }

    pub_map_.publish(line_list);
}

}

