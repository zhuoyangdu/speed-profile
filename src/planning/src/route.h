#ifndef PLANNING_SRC_ROUTE_H_
#define PLANNING_SRC_ROUTE_H_

#include <iostream>
#include "spline.h"
#include <ros/ros.h>
#include <common/string.h>
#include <common/file_config.h>

#include "planning_config.pb.h"

namespace planning {

class Route{
 public:
    Route();

    Spline get_x() {return curve_x_;}

    Spline get_y() {return curve_y_;}

    double GetCurvature(double s);

    double x(double s) {return curve_x_(s);}

    double y(double s) {return curve_y_(s);}

    double theta(double s) {return atan2(curve_x_.deriv1(s), curve_y_.deriv1(s));}

 private:

    void GetGeometryPath();

    std::string planning_path_;
    std::string road_file_;

    Spline curve_x_;
    Spline curve_y_;

};

} // namespace planning

#endif // PLANNING_SRC_ROUTE_H_
