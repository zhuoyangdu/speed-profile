#include "sumo/sumo_client.h"

SumoClient::SumoClient() {

}

bool SumoClient::Init(const std::string& host,
                      int port) {
    try {
        connect(host, port);
    } catch (tcpip::SocketException& e) {
        std::cout << "#Error while connecting: " << e.what() << std::endl;
        return false;
    }
    simulationStep(5000);
    GetVehicleList();
    return true;
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

