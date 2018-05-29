#include "route.h"
using namespace std;

namespace planning {

Route::Route() {
    ros::param::get("~planning_path", planning_path_);

    std::string file_name = planning_path_ + "/config/planning_config.pb.txt";
    planning::PlanningConfig planning_conf;
    if(!common::GetProtoFromASCIIFile(file_name, &planning_conf)) {
        ROS_ERROR("Error read config!");
    }
    road_file_ = planning_conf.road_file();

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

double Route::GetCurvature(double s) {
    double dx = curve_x_.deriv1(s);
    double ddx = curve_x_.deriv2(s);
    double dy = curve_y_.deriv1(s);
    double ddy = curve_y_.deriv2(s);
    double curvature = fabs(dx * ddy - ddx * dy) / pow(dx*dx+dy*dy, 1.5);
    return curvature;
}

}
