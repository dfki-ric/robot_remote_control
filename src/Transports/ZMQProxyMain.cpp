#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include <zmq.hpp>

int main(int argc, char** argv) {
    std::string ip;
    std::string robot_telemetryport;
    std::string local_telemetryport;

    if (argc == 1) {
        ip = "localhost";
        robot_telemetryport = "7002";
        local_telemetryport = "7003";
    } else if (argc == 4) {
        ip = argv[1];
        robot_telemetryport = argv[2];
        local_telemetryport = argv[3];
    } else {
        printf("needs 0 or 3 params: robot_ip robot_telemetryport local_telemetryport\n");
        exit(1);
    }

    printf("conencting to %s, ports: %s,%s\n", ip.c_str(), robot_telemetryport.c_str(), local_telemetryport.c_str());

    zmq::context_t context(1);
    zmq::socket_t sub(context, ZMQ_SUB);
    sub.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);  // subscribe all
    sub.connect("tcp://"+ip+":"+robot_telemetryport);


    zmq::socket_t pub(context, ZMQ_PUB);
    pub.bind("tcp://*:"+local_telemetryport);

    zmq::proxy(static_cast<void*>(sub), static_cast<void*>(pub), nullptr);

}
