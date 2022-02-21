#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include "ConsoleCommands.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

#define STRING(A) #A

// rrc_type defined outside of lamda to print latest values if no new ones are received
#define DEFINE_WATCH_COMMAND(TYPE, FUNCTION) \
    robot_remote_control::TYPE rrc_type_watch_##FUNCTION; \
    console.registerCommand("watch_" STRING(FUNCTION), [&](const std::vector<std::string> &params){ \
            if (controller.FUNCTION(&rrc_type_watch_##FUNCTION)) { \
                rrc_type_watch_##FUNCTION.PrintDebugString(); \
            } else { \
                usleep(10000); \
            } \
    }, true);

#define DEFINE_PRINT_COMMAND(TYPE, FUNCTION) \
    robot_remote_control::TYPE rrc_type_##FUNCTION; \
    console.registerCommand(#FUNCTION, [&](const std::vector<std::string> &params){ \
        bool received = false; \
        while (controller.FUNCTION(&rrc_type_##FUNCTION)) {received = true;} \
        rrc_type_##FUNCTION.PrintDebugString(); \
        if (!received) { \
            printf("no new data received \n"); \
        } \
    }); \
    DEFINE_WATCH_COMMAND(TYPE, FUNCTION)

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
    std::vector<ConsoleCommands::ParamDef> params;
    params.push_back(ConsoleCommands::ParamDef("name (string)", "name"));
    params.push_back(ConsoleCommands::ParamDef("value (float)", "0"));
    console.registerParamsForCommand("simpleaction", params);
    params.clear();

    console.registerCommand("requestsimpleactions", [&](const std::vector<std::string> &params){
        robot_remote_control::SimpleActions actions;
        controller.requestSimpleActions(&actions);
        // add param options to autocomplete
        std::vector<ConsoleCommands::ParamDef> simpleactionparams;
        simpleactionparams.push_back(ConsoleCommands::ParamDef("name (string)", "name"));
        simpleactionparams.push_back(ConsoleCommands::ParamDef("value (float)", "0"));
        for (auto &simpleaction : actions.actions()) {
            simpleactionparams[0].defaultvalues.push_back(simpleaction.name());
            // simpleactionparams[1].defaultvalues.push_back(simpleaction.state());
        }
        // replace generic params
        console.registerParamsForCommand("simpleaction", simpleactionparams);
        actions.PrintDebugString();
    });

    DEFINE_PRINT_COMMAND(Pose, getCurrentPose);
    DEFINE_PRINT_COMMAND(JointState, getCurrentJointState);
    DEFINE_PRINT_COMMAND(ContactPoints, getCurrentContactPoints);


    while (run) {
        if (!console.readline("rrc@" + ip + " $ ")) {
            run = false;
        }
    }

    printf("\n");
    fflush(stdout);

    return 0;
}
