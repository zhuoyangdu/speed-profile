#ifndef SUMOCLIENT_h
#define SUMOCLIENT_h

#include "sumo/utils/traci/TraCIAPI.h"
#include "sumo/foreign/tcpip/socket.h"

class SumoClient : public TraCIAPI
{
public:
    SumoClient();

    bool Init(const std::string& host, int port);

    void GetVehicleList();

private:

public:

private:

};

#endif // SUMOCLIENT_h
