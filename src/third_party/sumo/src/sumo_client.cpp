#include "sumo/sumo_client.h"
#include "sumo/utils/common/SUMOTime.h"
#include <unistd.h>

void SumoClient::Init(const std::string& host,
                      int port) {
    bool success_init = false;
    while(!success_init) {
        try {
            connect(host, port);
        } catch (tcpip::SocketException&) {
            if (!success_init) {
                std::cout << "#Error while connecting..." << std::endl;
                sleep(1);
            }
            continue;
        }
        success_init = true;
        std::cout << "Connected to sumo client." << std::endl;
    }

    //SUMOTime delta_t = 1000;
    //simulationStep(delta_t);
}

void SumoClient::GetVehicleList() {
    std::vector<std::string> vehicle_id_list =
        vehicle.getIDList();
    for (std::string id : vehicle_id_list) {
        std::cout << "ID: " << id;
        std::cout << " speed: " << vehicle.getSpeed(id);
        std::cout << " position: " << vehicle.getPosition(id).x << ", " << vehicle.getPosition(id).y;
        std::cout << " angle: " << vehicle.getAngle(id);
    }
}

