#include <iostream>
#include <stdlib.h>

#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include "ConsoleCommands.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;


// rrc_type defined outside of lamda to print latest values if no new ones are received
#define DEFINE_PRINT_COMMAND(TYPE, FUNCTION) \
    robot_remote_control::TYPE rrc_type; \
    console.registerCommand(#FUNCTION, [&](const std::vector<std::string> &params){ \
        bool received = false; \
        while(controller.FUNCTION(&rrc_type)){received = true;} \
        rrc_type.PrintDebugString(); \
        if (!received) { \
            printf("no new data received \n"); \
        } \
    });


int main(int argc, char** argv) {
    printf("\nThis is work in progress, not a functional CLI\n\n");

    std::string ip;
    std::string commandport;
    std::string telemetryport;

    if (argc == 1) {
        ip = "localhost";
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

    // set Heartbeat to one second
    controller.setHeartBeatDuration(1);

    bool run = true;

    console.registerCommand("stats", [&](const std::vector<std::string> &params){
        controller.getStatistics().calculate();
        controller.getStatistics().print(true);
    });

    console.registerCommand("exit", [&](const std::vector<std::string> &params){
        printf("\n");
        fflush(stdout);
        run = false;
    });

    console.registerCommand("simpleaction", [&](const std::vector<std::string> &params){
        std::for_each(params.begin(), params.end(), [](const std::string &param) {
            std::cout << param << std::endl;
        });
        robot_remote_control::SimpleAction action;
        action.set_name(params[0]);
        try {
            action.set_state(std::stof(params[1]));
        } catch (const std::invalid_argument &e) {
            std::cout << "Simpleaction state must be a number, was '" << params[1] <<"' " << std::endl;
            std::cout << e.what() << std::endl;
            return;
        }
        controller.setSimpleActionCommand(action);
    });

    console.registerCommand("requestsimpleactions", [&](const std::vector<std::string> &params){
        robot_remote_control::SimpleActions actions;
        controller.requestSimpleActions(&actions);
        //TODO: add to autocomplete
        actions.PrintDebugString();
    });

    DEFINE_PRINT_COMMAND(ContactPoints, getCurrentContactPoints);

    //define outside to keep current
    // robot_remote_control::ContactPoints points;
    // console.registerCommand("printContactPoints", [&](const std::vector<std::string> &params){
    //     bool received = false;
    //     while(controller.getCurrentContactPoints(&points)){received = true;}
    //     points.PrintDebugString();
    //     if (!received) {
    //         printf("no new data received \n");
    //     }
    // });


    while (run) {
        console.readline("rrc@" + ip +" $ ");
    }

    return 0;
}
