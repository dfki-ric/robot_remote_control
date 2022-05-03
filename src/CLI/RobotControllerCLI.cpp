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
#define DEFINE_WATCH_COMMAND(TYPE, FUNCTION, DOC) \
    robot_remote_control::TYPE rrc_type_watch_##FUNCTION; \
    console.registerCommand("watch_" STRING(FUNCTION), DOC, [&](const std::vector<std::string> &params) { \
            if (controller.FUNCTION(&rrc_type_watch_##FUNCTION)) { \
                rrc_type_watch_##FUNCTION.PrintDebugString(); \
            } else { \
                usleep(10000); \
            } \
    }, true);

#define DEFINE_PRINT_COMMAND(TYPE, FUNCTION, DOC) \
    robot_remote_control::TYPE rrc_type_##FUNCTION; \
    console.registerCommand(#FUNCTION, DOC, [&](const std::vector<std::string> &params) { \
        bool received = false; \
        while (controller.FUNCTION(&rrc_type_##FUNCTION)) {received = true;} \
        rrc_type_##FUNCTION.PrintDebugString(); \
        if (!received) { \
            printf("no new data received \n"); \
        } \
    }); \
    DEFINE_WATCH_COMMAND(TYPE, FUNCTION, "continuously "#DOC", press Enter to stop")

#define DEFINE_REQUEST_COMMAND(TYPE, FUNCTION, DOC) \
    robot_remote_control::TYPE rrc_type_##FUNCTION; \
    console.registerCommand(#FUNCTION, DOC, [&](const std::vector<std::string> &params) { \
        if (controller.FUNCTION(&rrc_type_##FUNCTION)) { \
            rrc_type_##FUNCTION.PrintDebugString(); \
        } else { \
            printf("no new data received \n"); \
        } \
    });

int main(int argc, char** argv) {
    printf("\nThis is work in progress, not a fully functional CLI.\nUse TAB to show/complete commands, type 'exit' or use crtl-d to close\n\n");

    std::string ip;
    std::string commandport;
    std::string telemetryport;

    if (argc == 1) {
        ip = "localhost";
        commandport = "7001";
        telemetryport = "7002";
    } else if (argc == 2) {
        ip = argv[1];
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

    printf("connecting to %s, ports: %s,%s\n", ip.c_str(), commandport.c_str(), telemetryport.c_str());

    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://"+ip+":"+commandport, TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://"+ip+":"+telemetryport, TransportZmq::SUB));

    robot_remote_control::RobotController controller(commands, telemetry);
    controller.startUpdateThread(0);

    ConsoleCommands console;
    std::vector<ConsoleCommands::ParamDef> params;

    // set Heartbeat to one second
    controller.setHeartBeatDuration(1);

    bool run = true;

    console.registerCommand("help", "display this help" , [&](const std::vector<std::string> &params){
        console.printHelp();
        printf("\nUse TAB to show/complete commands, type 'exit' or use crtl-d to close\n\n");
    });

    console.registerCommand("exit", "exit this CLI", [&](const std::vector<std::string> &params){
        printf("\n");
        fflush(stdout);
        run = false;
    });

    console.registerCommand("clear", "clear console display", [&](const std::vector<std::string> &params){
        int exit_code = system("clear");
    });

    console.registerCommand("stats", "show telemetry statistics", [&](const std::vector<std::string> &params){
        controller.getStatistics().calculate();
        controller.getStatistics().print(true);
    });

    /**
     * generic defines
     */
    DEFINE_PRINT_COMMAND(Pose, getCurrentPose, "print current Pose");
    DEFINE_PRINT_COMMAND(JointState, getCurrentJointState, "print current JointState");
    DEFINE_PRINT_COMMAND(ContactPoints, getCurrentContactPoints, "print current ContactPoints");
    DEFINE_PRINT_COMMAND(IMU, getCurrentIMUState, "print current IMU");
    DEFINE_PRINT_COMMAND(Odometry, getOdometry, "print current Odometry");
    DEFINE_PRINT_COMMAND(RobotState, getRobotState, "print current Robot state");


    DEFINE_REQUEST_COMMAND(ControllableFrames, requestControllableFrames, "print ControllableFrames set by the robot");
    DEFINE_REQUEST_COMMAND(JointState, requestControllableJoints, "print ControllableJoints set by the robot");
    DEFINE_REQUEST_COMMAND(RobotName, requestRobotName, "print Robot name");
    DEFINE_REQUEST_COMMAND(VideoStreams, requestVideoStreams, "print video Stream ulrs");
    DEFINE_REQUEST_COMMAND(RobotState, requestRobotState, "print current Robot state");
    

    /**
     * Commands
     */

    console.registerCommand("setTwistCommand", "execute a twist", [&](const std::vector<std::string> &params){
        robot_remote_control::Twist twist;
        int i = 0;
        try {
            twist.mutable_linear()->set_x(std::stof(params[i++]));
            twist.mutable_linear()->set_y(std::stof(params[i++]));
            twist.mutable_linear()->set_z(std::stof(params[i++]));
            twist.mutable_angular()->set_x(std::stof(params[i++]));
            twist.mutable_angular()->set_y(std::stof(params[i++]));
            twist.mutable_angular()->set_z(std::stof(params[i++]));
        } catch (const std::invalid_argument &e) {
            std::cout << "value must be a number, was '" << params[i] <<"' " << std::endl;
            std::cout << e.what() << std::endl;
            return;
        }
        controller.setTwistCommand(twist);
    });

    params.push_back(ConsoleCommands::ParamDef("linear x (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("linear y (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("linear z (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular x (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular y (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular z (float)", "0"));
    console.registerParamsForCommand("setTwistCommand", params);
    params.clear();

    console.registerCommand("setJointCommand", "send joint position command", [&](const std::vector<std::string> &params){
        robot_remote_control::JointCommand cmd;
        try {
            cmd.add_name(params[0]);
            cmd.add_position(std::stof(params[1]));
        } catch (const std::invalid_argument &e) {
            std::cout << "second value must be a number, was '" << params[1] <<"' " << std::endl;
            std::cout << e.what() << std::endl;
            return;
        }
        controller.setJointCommand(cmd);
    });

    params.push_back(ConsoleCommands::ParamDef("joint_name (string)", "empty"));
    params.push_back(ConsoleCommands::ParamDef("position (float)", "0"));
    console.registerParamsForCommand("setJointCommand", params);
    params.clear();

    console.registerCommand("setSimpleActionCommand", "execute a simpleaction", [&](const std::vector<std::string> &params){
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
    params.push_back(ConsoleCommands::ParamDef("name (string)", "name"));
    params.push_back(ConsoleCommands::ParamDef("value (float)", "0"));
    console.registerParamsForCommand("setSimpleActionCommand", params);
    params.clear();

    console.registerCommand("requestSimpleActions", "request simple actions and add them to autocomplete", [&](const std::vector<std::string> &params){
        robot_remote_control::SimpleActions actions;
        if (controller.requestSimpleActions(&actions)) {
            actions.PrintDebugString();
            // add param options to autocomplete
            for (auto &simpleaction : actions.actions()) {
                console.addParamDefaultValue("setSimpleActionCommand", 0, simpleaction.name());
            }
        } else {
            printf("no new data received \n");
        }
    });


    /**
     * Special telemetry functions (non-default print etc.)
     */

    robot_remote_control::Image rrc_type_image;
    console.registerCommand("getImage", "get a single image and print its properties (without data)", [&](const std::vector<std::string> &params) {
        bool received = false;
        while (controller.getImage(&rrc_type_image)) {received = true;}
        printf("image type: %s (%ix%i) frame: %s size: %lu bytes\n", rrc_type_image.encoding().c_str(), rrc_type_image.width(), rrc_type_image.height(), rrc_type_image.header().frame().c_str(), rrc_type_image.data().size());
        if (!received) {
            printf("no new data received \n");
        }
    });
    console.registerCommand("watch_getImage", "get a single image and print its properties (without data)", [&](const std::vector<std::string> &params){
        robot_remote_control::Image rrc_type_image;
        while (controller.getImage(&rrc_type_image)) {
            printf("image type: %s (%ix%i) frame: %s size: %lu bytes\n", rrc_type_image.encoding().c_str(), rrc_type_image.width(), rrc_type_image.height(), rrc_type_image.header().frame().c_str(), rrc_type_image.data().size());
        }
    }, true);

    robot_remote_control::PointCloud rrc_type_pointcloud;
    console.registerCommand("getPointCloud", "get a single pointcloud and print its properties (without data)", [&](const std::vector<std::string> &params) {
        bool received = false;
        while (controller.getPointCloud(&rrc_type_pointcloud)) {
            received = true;
            printf("pointcloud in frame %s with %i points\n", rrc_type_pointcloud.header().frame().c_str(), rrc_type_pointcloud.points().size());
        }
        if (!received) {
            printf("no new data received \n");
        }
    });
    console.registerCommand("watch_getPointCloud", "get a single pointcloud and print its properties (without data)", [&](const std::vector<std::string> &params){
        robot_remote_control::PointCloud rrc_type_pointcloud;
        while (controller.getPointCloud(&rrc_type_pointcloud)) {
            printf("pointcloud in frame %s with %i points\n", rrc_type_pointcloud.header().frame().c_str(), rrc_type_pointcloud.points().size());
        }
    }, true);


    /**
     * Files
     */
    console.registerCommand("requestAvailableFiles", "print files available for download", [&](const std::vector<std::string> &params){
        robot_remote_control::FileDefinition files;
        if (controller.requestAvailableFiles(&files)) {
            files.PrintDebugString();
            // add param options to autocomplete
            for (auto &file : files.file()) {
                console.addParamDefaultValue("requestFile", 0, file.identifier());
            }
        } else {
            printf("no new data received \n");
        }
    });

    console.registerCommand("requestFile", "download a file", [&](const std::vector<std::string> &params){
        if (controller.requestFile(params[0], std::stoi(params[1]), params[2])) {
            printf("files written\n");
        } else {
            printf("no files received\n");
        }
    });
    params.push_back(ConsoleCommands::ParamDef("identifier (string)", ""));
    params.push_back(ConsoleCommands::ParamDef("compressed (bool)", "1"));
    params.push_back(ConsoleCommands::ParamDef("target path (string)", "./"));
    console.registerParamsForCommand("requestFile", params);
    params.clear();

    /**
     * Main loop
     */

    while (run) {
        if (!console.readline("rrc@" + ip + " $ ")) {
            run = false;
        }
    }

    printf("\n");
    fflush(stdout);

    return 0;
}
