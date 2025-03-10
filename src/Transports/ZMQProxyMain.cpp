#include <iostream>
#include <thread>

#include <stdlib.h>
#include <unistd.h>

#include <zmq.hpp>

/**
 * @brief This proxy can be used to only maintain a single connection to the robot while havinf several local ones
 */
int main(int argc, char** argv) {
    std::string robotip;
    std::string robot_commandport;
    std::string local_commandport;
    std::string robot_telemetryport;
    std::string local_telemetryport;

    if (argc == 1) {
        robotip = "localhost";
        robot_commandport = "7001";
        local_commandport = "7001";
        robot_telemetryport = "7002";
        local_telemetryport = "7002";
    } else if (argc == 2) {
        robotip = argv[1];
        robot_commandport = "7001";
        local_commandport = "7001";
        robot_telemetryport = "7002";
        local_telemetryport = "7002";
    } else if (argc == 4) {
        robotip = argv[1];
        robot_telemetryport = argv[2];
        local_telemetryport = argv[3];
        robot_telemetryport = argv[4];
        local_telemetryport = argv[5];
    } else {
        printf("needs 0, 1, or 5 params: [robot_ip] [robot_commandport robot_telemetryport local_commandport local_telemetryport]\n");
        exit(1);
    }

    printf("proxy connecting to %s, ports: %s,%s, listening on: %s,%s (commands,telemetry)\n", robotip.c_str(), robot_commandport.c_str(), robot_telemetryport.c_str(), local_commandport.c_str(), local_telemetryport.c_str());

    std::thread tememetrythread = std::thread([&](){
        zmq::context_t context(4);
        zmq::socket_t sub(context, ZMQ_SUB);

        #ifndef ZMQ_CPP11
            sub.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);  // subscribe all
        #else
            sub.set(zmq::sockopt::subscribe, "");
        #endif
        sub.connect("tcp://"+robotip+":"+robot_telemetryport);


        zmq::socket_t pub(context, ZMQ_PUB);
        pub.bind("tcp://*:"+local_telemetryport);

        // zmq::proxy(static_cast<void*>(sub), static_cast<void*>(pub), nullptr);
        zmq::proxy(sub, pub);
    });

    zmq::context_t context(1);
    zmq::socket_t req(context, ZMQ_REQ);
    req.connect("tcp://"+robotip+":"+robot_commandport);


    zmq::socket_t rep(context, ZMQ_REP);
    rep.bind("tcp://*:"+local_commandport);

    //zmq::proxy(static_cast<void*>(req), static_cast<void*>(rep), nullptr);
    zmq::proxy(req, rep);


    tememetrythread.join();
}
