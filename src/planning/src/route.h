#ifndef PLANNING_SRC_ROUTE_H_
#define PLANNING_SRC_ROUTE_H_

#include <iostream>
#include "spline.h"
#include <ros/ros.h>
#include <common/string.h>

namespace planning {

class Route{
 public:
    Route();

    Spline get_x() {return curve_x_;}

    Spline get_y() {return curve_y_;}

 private:

    void GetGeometryPath();

    std::string planning_path_;
    std::string road_file_;

    Spline curve_x_;
    Spline curve_y_;

};

} // namespace planning

#endif // PLANNING_SRC_ROUTE_H_
