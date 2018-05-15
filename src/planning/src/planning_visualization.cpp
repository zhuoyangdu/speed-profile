#include "planning_visualization.h"

using namespace std;

namespace planning {
PlanningVisualization::PlanningVisualization() {
    InitPublishers();
}

void PlanningVisualization::InitPublishers() {
    pub_vehicle_state_vis_ =
        nh_.advertise<visualization_msgs::Marker>(
        "/planning_vis/vehicle_state", rate_);
}

}

