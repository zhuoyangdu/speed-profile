#include <vector>
#include <string>

namespace simulation {
class SimulationConf {
 public:
    SimulationConf() {
        ReadParam();
    }

 private:
    void ReadParam() {
        ros::param::get("~remote_port", remote_port);
    }

 public:
    int remote_port = 1337;
};

}
