#include "route.h"

namespace planning {

Route::Route() {
    ros::param::get("~planning_path", planning_path_);
    ros::param::get("~road_file", road_file_);

    GetGeometryPath();
}

void Route::GetGeometryPath() {
    std::vector<double> xs, ys;
    std::string line;
    std::string file_name = planning_path_ + "/../simulation/data/path/" + road_file_;
    std::ifstream file(file_name);
    if (file.is_open()) {
        int i;
        while (getline(file, line)) {
            std::vector<std::string> ps;
            common::StringUtils string_utils;
            string_utils.SplitString(line, ",", &ps);
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
    Spline::fitCurve(xs, ys, &curve_x_, &curve_y_, &path_length);
}

}
