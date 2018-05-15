#ifndef SUMOCLIENT_h
#define SUMOCLIENT_h

#include "sumo/utils/traci/TraCIAPI.h"
#include "sumo/foreign/tcpip/socket.h"

class SumoClient : public TraCIAPI
{
public:
    SumoClient() {};

    ~SumoClient() { close(); }
    void Init(const std::string& host, int port);

    void GetVehicleList();

    void SetEgoVehicle();

    void GetEgoVehicleState();

private:

public:

private:

};

#endif // SUMOCLIENT_h
