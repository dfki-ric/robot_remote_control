#include <iostream>
#include <stdlib.h>

#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include "ConsoleCommands.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

int main(int argc, char** argv) {
    printf("\nThis is work in progress, not a functional CLI\n\n");

    std::string ip(argv[1]);
    std::string commandport(argv[2]);
    std::string telemetryport(argv[3]);

    if (argc == 1) {
        ip = "127.0.0.1";
        commandport = "7001";
        telemetryport = "7002";
    } else if (argc == 4) {
        ip = argv[1];
        commandport = argv[2];
        telemetryport = argv[3];
    } else {
        printf("needs 0 or 3 params: ip commandport telemetryport\n");
        exit(1);
    }

    printf("conencting to %s, ports: %s,%s\n", ip.c_str(), commandport.c_str(), telemetryport.c_str());

    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://"+ip+":"+commandport, TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://"+ip+":"+telemetryport, TransportZmq::SUB));

    robot_remote_control::RobotController controller(commands, telemetry);
    controller.startUpdateThread(0);

    ConsoleCommands console;

    bool run = true;

    console.registerCommand("test", [&](){
        printf("test done\n");
    });

    console.registerCommand("stats", [&](){
        controller.getStatistics().calculate();
        controller.getStatistics().print(true);
    });

    console.registerCommand("exit", [&](){
        printf("\n");
        fflush(stdout);
        run = false;
    });


    while (run) {
        console.readline("rrc@" + ip +" $ ");
    }

    return 0;
}
