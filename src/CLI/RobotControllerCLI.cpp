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
    console.registerCommand("watch_" STRING(FUNCTION), DOC, [&](const std::vector<std::string> &params) -> bool { \
            if (controller.FUNCTION(&rrc_type_watch_##FUNCTION)) { \
                rrc_type_watch_##FUNCTION.PrintDebugString(); \
            } else { \
                usleep(10000); \
            } \
            return true; \
    }, true);

#define DEFINE_PRINT_COMMAND(TYPE, FUNCTION, DOC) \
    robot_remote_control::TYPE rrc_type_##FUNCTION; \
    console.registerCommand(#FUNCTION, DOC, [&](const std::vector<std::string> &params) -> bool { \
        if (controller.FUNCTION(&rrc_type_##FUNCTION, true)) { \
            rrc_type_##FUNCTION.PrintDebugString(); \
            return true; \
        } else { \
            printf("no new data received \n"); \
            return false; \
        } \
    }); \
    DEFINE_WATCH_COMMAND(TYPE, FUNCTION, "continuously "#DOC", press Enter to stop")

#define DEFINE_REQUEST_COMMAND(TYPE, FUNCTION, DOC) \
    robot_remote_control::TYPE rrc_type_##FUNCTION; \
    console.registerCommand(#FUNCTION, DOC, [&](const std::vector<std::string> &params) -> bool { \
        if (controller.FUNCTION(&rrc_type_##FUNCTION)) { \
            rrc_type_##FUNCTION.PrintDebugString(); \
            return true; \
        } else { \
            printf("no new data received \n"); \
            return false; \
        } \
    });

int main(int argc, char** argv) {
    printf("\nThis is work in progress, not a fully functional CLI.\nUse TAB to show/complete commands, type 'exit' or use crtl-d to close\n\n");

    std::string ip;
    std::string commandport;
    std::string telemetryport;
    bool SUCCESS = true;

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
        return true;
    });

    console.registerCommand("exit", "exit this CLI", [&](const std::vector<std::string> &params){
        printf("\n");
        fflush(stdout);
        run = false;
        return true;
    });

    console.registerCommand("clear", "clear console display", [&](const std::vector<std::string> &params){
        int exit_code = system("clear");
        return true;
    });

    console.registerCommand("stats", "show telemetry statistics", [&](const std::vector<std::string> &params){
        controller.getStatistics().calculate();
        controller.getStatistics().print(true);
        return true;
    });

    /**
     * generic defines
     */
    DEFINE_PRINT_COMMAND(Pose, getCurrentPose, "print current Pose");
    DEFINE_PRINT_COMMAND(JointState, getCurrentJointState, "print current JointState");
    DEFINE_PRINT_COMMAND(ContactPoints, getCurrentContactPoints, "print current ContactPoints");
    DEFINE_PRINT_COMMAND(IMU, getCurrentIMUState, "print current IMU");
    DEFINE_PRINT_COMMAND(Odometry, getOdometry, "print current Odometry");


    DEFINE_REQUEST_COMMAND(ControllableFrames, requestControllableFrames, "print ControllableFrames set by the robot");
    DEFINE_REQUEST_COMMAND(RobotName, requestRobotName, "print Robot name");
    DEFINE_REQUEST_COMMAND(VideoStreams, requestVideoStreams, "print video Stream ulrs");
    DEFINE_REQUEST_COMMAND(RobotState, requestRobotState, "print current Robot state");


    /**
     * @brief specialized getRobotState (no optional onlyNewest flag)
     */
    robot_remote_control::RobotState rrc_type_getRobotState;
    console.registerCommand("getRobotState", "print current Robot state", [&](const std::vector<std::string> &params) {
        bool received = false;
        while (controller.getRobotState(&rrc_type_requestRobotState)) {
            received = true;
            rrc_type_requestRobotState.PrintDebugString();
        }
        if (!received){
            printf("no new data received \n");
        }
        return received;
    });
    DEFINE_WATCH_COMMAND(RobotState, getRobotState, "continuously print current Robot state, press Enter to stop")


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
            return false;
        }
        controller.setTwistCommand(twist);
        return true;
    });

    params.push_back(ConsoleCommands::ParamDef("linear x (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("linear y (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("linear z (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular x (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular y (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular z (float)", "0"));
    console.registerParamsForCommand("setTwistCommand", params);
    params.clear();

    console.registerCommand("setTargetPose3D", "send target pose command", [&](const std::vector<std::string> &params) {
        robot_remote_control::Pose pose;
        int i = 0;
        try {
            pose.mutable_position()->set_x(std::stof(params[i++]));
            pose.mutable_position()->set_y(std::stof(params[i++]));
            pose.mutable_position()->set_z(std::stof(params[i++]));
            pose.mutable_orientation()->set_x(std::stof(params[i++]));
            pose.mutable_orientation()->set_y(std::stof(params[i++]));
            pose.mutable_orientation()->set_z(std::stof(params[i++]));
            pose.mutable_orientation()->set_w(std::stof(params[i++]));
            pose.mutable_header()->set_frame(params[i++]);
        } catch (const std::invalid_argument &e) {
            std::cout << "value must be a number, was '" << params[i] <<"' " << std::endl;
            std::cout << e.what() << std::endl;
            return false;
        }
        controller.setTargetPose(pose);
        return true;
    });
    params.push_back(ConsoleCommands::ParamDef("pos x (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("pos y (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("pos z (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("orientation x (float, quaternion)", "0"));
    params.push_back(ConsoleCommands::ParamDef("orientation y (float, quaternion)", "0"));
    params.push_back(ConsoleCommands::ParamDef("orientation z (float, quaternion)", "0"));
    params.push_back(ConsoleCommands::ParamDef("orientation w (float, quaternion)", "1"));
    params.push_back(ConsoleCommands::ParamDef("frame (string)", "base"));
    console.registerParamsForCommand("setTargetPose3D", params);
    params.clear();

    console.registerCommand("setJointCommand", "send joint position command", [&](const std::vector<std::string> &params){
        robot_remote_control::JointCommand cmd;
        try {
            cmd.add_name(params[0]);
            cmd.add_position(std::stof(params[1]));
        } catch (const std::invalid_argument &e) {
            std::cout << "second value must be a number, was '" << params[1] <<"' " << std::endl;
            std::cout << e.what() << std::endl;
            return false;
        }
        controller.setJointCommand(cmd);
        return true;
    });

    params.push_back(ConsoleCommands::ParamDef("joint_name (string)", "empty"));
    params.push_back(ConsoleCommands::ParamDef("position (float)", "0"));
    console.registerParamsForCommand("setJointCommand", params);
    params.clear();

    console.registerCommand("requestControllableJoints", "print ControllableJoints set by the robot", [&](const std::vector<std::string> &params){
        robot_remote_control::JointState joints;
        if (controller.requestControllableJoints(&joints)) {
            joints.PrintDebugString();
            // add param options to autocomplete
            for (auto &jointname : joints.name()) {
                console.addParamDefaultValue("setJointCommand", 0, jointname);
            }
            return true;
        } else {
            printf("no new data received \n");
            return false;
        }
    });

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
            return false;
        }
        controller.setSimpleActionCommand(action);
        return true;
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
            return true;
        } else {
            printf("no new data received \n");
            return false;
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
        return received;
    });
    console.registerCommand("watch_getImage", "get a single image and print its properties (without data)", [&](const std::vector<std::string> &params){
        robot_remote_control::Image rrc_type_image;
        while (controller.getImage(&rrc_type_image)) {
            printf("image type: %s (%ix%i) frame: %s size: %lu bytes\n", rrc_type_image.encoding().c_str(), rrc_type_image.width(), rrc_type_image.height(), rrc_type_image.header().frame().c_str(), rrc_type_image.data().size());
        }
        return true;
    }, true);

    robot_remote_control::PointCloud rrc_type_pointcloud;
    console.registerCommand("getPointCloud", "get a single pointcloud and print its properties (without data)", [&](const std::vector<std::string> &params) {
        bool received = false;
        while (controller.getPointCloud(&rrc_type_pointcloud)) {
            received = true;
            rrc_type_pointcloud.header().PrintDebugString();
            rrc_type_pointcloud.origin().PrintDebugString();
            printf("pointcloud with %i points and %i channels\n", rrc_type_pointcloud.points().size(), rrc_type_pointcloud.channels().size());
        }
        if (!received) {
            printf("no new data received \n");
        }
        return received;
    });
    console.registerCommand("watch_getPointCloud", "get a single pointcloud and print its properties (without data)", [&](const std::vector<std::string> &params){
        robot_remote_control::PointCloud rrc_type_pointcloud;
        while (controller.getPointCloud(&rrc_type_pointcloud)) {
            rrc_type_pointcloud.header().PrintDebugString();
            rrc_type_pointcloud.origin().PrintDebugString();
            printf("pointcloud with %i points and %i channels\n", rrc_type_pointcloud.points().size(), rrc_type_pointcloud.channels().size());
        }
        return true;
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
            return true;
        } else {
            printf("no new data received \n");
        }
        return false;
    });

    console.registerCommand("requestFile", "download a file", [&](const std::vector<std::string> &params){
        if (controller.requestFile(params[0], std::stoi(params[1]), params[2])) {
            printf("files written\n");
            return true;
        } else {
            printf("no files received\n");
            return false;
        }
    });
    params.push_back(ConsoleCommands::ParamDef("identifier (string)", ""));
    params.push_back(ConsoleCommands::ParamDef("compressed (bool)", "1"));
    params.push_back(ConsoleCommands::ParamDef("target path (string)", "./"));
    console.registerParamsForCommand("requestFile", params);
    params.clear();

    /**
     * Simeplesensors
     */
    console.registerCommand("getSimpleSensor", "get simple sensor", [&](const std::vector<std::string> &params){
        robot_remote_control::SimpleSensor simplesensor;
        // simplesensores have a buffer of size 1
        if (controller.getSimpleSensor(std::stoi(params[0]), &simplesensor)) {
            if (simplesensor.value().size() > 25) {
                // delete data (so it is not printed)
                simplesensor.mutable_value()->Clear();
            }
            simplesensor.PrintDebugString();
            return true;
        } else {
            printf("no files received\n");
            return false;
        }
    });
    params.push_back(ConsoleCommands::ParamDef("id (int)", ""));
    console.registerParamsForCommand("getSimpleSensor", params);
    params.clear();


    /**
     * Main loop
     */
    controller.waitForConnection();
    while (run && SUCCESS) {
        SUCCESS = console.readline("rrc@" + ip + " $ ");
    }

    printf("\n");
    fflush(stdout);

    return not SUCCESS;
}
