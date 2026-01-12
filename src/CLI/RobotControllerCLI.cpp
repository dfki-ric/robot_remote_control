#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include "ConsoleCommands.hpp"

#include <boost/program_options.hpp>

#ifdef ADD_WEBSOCKET_TRANSPORT
    #include "Transports/TransportWebSocket.hpp"
    using robot_remote_control::TransportWebSocket;
#endif

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

#define STRING(A) #A

// rrc_type defined outside of lamda to print latest values if no new ones are received
#define DEFINE_WATCH_COMMAND(TYPE, FUNCTION, CHANNEL, DOC) \
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
        uint8_t channel = 0; \
        if (params.size()) { \
            channel = std::atoi(params.front().c_str()); \
        } \
        if (controller.FUNCTION(&rrc_type_##FUNCTION, true, channel)) { \
            rrc_type_##FUNCTION.PrintDebugString(); \
            return true; \
        } else { \
            printf("no new data received \n"); \
            return false; \
        } \
    }); \
    { \
        std::vector<ConsoleCommands::ParamDef> params; \
        params.push_back(ConsoleCommands::ParamDef("channel", "0", true)); \
        console.registerParamsForCommand(#FUNCTION, params); \
    } \
    DEFINE_WATCH_COMMAND(TYPE, FUNCTION, channel, "continuously "#DOC", press Enter to stop")

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
    std::string ip = "127.0.0.1";
    std::string commandport = "7001";
    std::string telemetryport = "7002";
    bool SUCCESS = true;
    bool EXIT_ON_FAILURE = false;
    bool ws = false;
    bool json = false;

    boost::program_options::options_description options_desc("Options:");
    options_desc.add_options()
        ("help,h", "print help")
        ("ip,i", boost::program_options::value(&ip), "The ip or host to conenct to")
        ("commandport,c", boost::program_options::value(&commandport), "the port to connect to for commands")
        ("telemetryport,t", boost::program_options::value(&telemetryport), "the port to connect to for telemetry")
        ("json", boost::program_options::bool_switch(&json), "use JSON serialization")
        #ifdef ADD_WEBSOCKET_TRANSPORT
        ("ws", boost::program_options::bool_switch(&ws), "connect via websocket instead of ZeroMQ")
        #endif
        ;

    boost::program_options::positional_options_description p;
    p.add("ip", 1);
    p.add("commandport", 2);
    p.add("telemetryport", 3);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                    .options(options_desc).positional(p)
                                    .style(boost::program_options::command_line_style::unix_style ^ boost::program_options::command_line_style::allow_short)
                                    .run(), vm
                                 );
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << "Usage: $> robot_controller [options] host commandport telemetryport" << std::endl;
        std::cout << options_desc << std::endl;
        return 0;
    }

    printf("\nThis is work in progress, not a fully functional CLI.\nUse TAB to show/complete commands, type 'exit' or use crtl-d to close\n\n");

    TransportSharedPtr commands;
    TransportSharedPtr telemetry;

    if (ws) {
        printf("connecting WebSocket to %s, ports: %s,%s\n", ip.c_str(), commandport.c_str(), telemetryport.c_str());
        commands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT, 7001, "localhost"));
        telemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT, 7002, "localhost"));
    } else {
        printf("connecting ZMQ to %s, ports: %s,%s\n", ip.c_str(), commandport.c_str(), telemetryport.c_str());
        commands = TransportSharedPtr(new TransportZmq("tcp://"+ip+":"+commandport, TransportZmq::REQ));
        telemetry = TransportSharedPtr(new TransportZmq("tcp://"+ip+":"+telemetryport, TransportZmq::SUB));
    }

    robot_remote_control::RobotController controller(commands, telemetry);

    if (json) {
        controller.setSerializationMode(robot_remote_control::Serialization::JSON);
    }

    controller.startUpdateThread(10);

    ConsoleCommands console;
    std::vector<ConsoleCommands::ParamDef> params;

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

    console.registerCommand("exit_on_failure", "used for parsing text files for automatic api tests", [&](const std::vector<std::string> &params){
        EXIT_ON_FAILURE=true;
        return true;
    });

    console.registerCommand("sleep", "sleep for a defined amount of seconds. used for file parsing.", [&](const std::vector<std::string> &params){
        usleep(std::stof(params[0])*1000000);
        return true;
    });
    params.push_back(ConsoleCommands::ParamDef("seconds (float)", "0"));
    console.registerParamsForCommand("sleep", params);
    params.clear();


    /**
     * generic defines
     */
    DEFINE_PRINT_COMMAND(Pose, getCurrentPose, "print current Pose");
    DEFINE_PRINT_COMMAND(Poses, getPoses, "print currently available Poses");
    DEFINE_PRINT_COMMAND(JointState, getCurrentJointState, "print current JointState");
    DEFINE_PRINT_COMMAND(ContactPoints, getCurrentContactPoints, "print current ContactPoints");
    DEFINE_PRINT_COMMAND(IMU, getCurrentIMUState, "print current IMU");
    DEFINE_PRINT_COMMAND(Odometry, getOdometry, "print current Odometry");
    DEFINE_PRINT_COMMAND(Transforms, getCurrentTransforms, "print current Transforms");
    DEFINE_PRINT_COMMAND(SimpleSensor, getSimpleSensor, "print simple sensor content");
    DEFINE_PRINT_COMMAND(WrenchState, getCurrentWrenchState, "print wrench data");




    DEFINE_REQUEST_COMMAND(ControllableFrames, requestControllableFrames, "print ControllableFrames set by the robot");
    DEFINE_REQUEST_COMMAND(RobotName, requestRobotName, "print Robot name");
    DEFINE_REQUEST_COMMAND(VideoStreams, requestVideoStreams, "print video Stream ulrs");
    DEFINE_REQUEST_COMMAND(RobotState, requestRobotState, "print current Robot state");
    DEFINE_REQUEST_COMMAND(ChannelsDefinition, requestChannelsDefinition, "get channels defined by the robot")

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
        if (!received) {
            printf("no new data received \n");
        }
        return received;
    });
    DEFINE_WATCH_COMMAND(RobotState, getRobotState, 0, "continuously print current Robot state, press Enter to stop")


    /**
     * Commands
     */

    auto processTwist = [](const std::vector<std::string> &params) {
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
        }
        if (params.size() > 6){
            twist.mutable_header()->set_frame(params[i++]);
        }
        return twist;
    };

    console.registerCommand("setTwistCommand", "execute a twist", [&](const std::vector<std::string> &params){
        robot_remote_control::Twist twist = processTwist(params);
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


    console.registerCommand("setTwistCommandForFrame", "execute a twist", [&](const std::vector<std::string> &params){
        robot_remote_control::Twist twist = processTwist(params);
        controller.setTwistCommand(twist);
        return true;
    });
    params.push_back(ConsoleCommands::ParamDef("linear x (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("linear y (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("linear z (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular x (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular y (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("angular z (float)", "0"));
    params.push_back(ConsoleCommands::ParamDef("frame name", ""));
    console.registerParamsForCommand("setTwistCommandForFrame", params);
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

    console.registerCommand("setGoToCommand", "send target pose command", [&](const std::vector<std::string> &params) {
        robot_remote_control::GoTo pose;
        int i = 0;
        try {
            pose.mutable_waypoint_pose()->mutable_position()->set_x(std::stof(params[i++]));
            pose.mutable_waypoint_pose()->mutable_position()->set_y(std::stof(params[i++]));
            pose.mutable_waypoint_pose()->mutable_position()->set_z(std::stof(params[i++]));
            pose.mutable_waypoint_pose()->mutable_orientation()->set_x(std::stof(params[i++]));
            pose.mutable_waypoint_pose()->mutable_orientation()->set_y(std::stof(params[i++]));
            pose.mutable_waypoint_pose()->mutable_orientation()->set_z(std::stof(params[i++]));
            pose.mutable_waypoint_pose()->mutable_orientation()->set_w(std::stof(params[i++]));
            pose.mutable_header()->set_frame(params[i++]);
            pose.set_max_forward_speed(std::stof(params[i++]));
            pose.set_waypoint_max_forward_speed(std::stof(params[i++]));
        } catch (const std::invalid_argument &e) {
            std::cout << "value must be a number, was '" << params[i] <<"' " << std::endl;
            std::cout << e.what() << std::endl;
            return false;
        }
        controller.setGoToCommand(pose);
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
    params.push_back(ConsoleCommands::ParamDef("max speed (float)", "1"));
    params.push_back(ConsoleCommands::ParamDef("arrive speed (float)", "0"));
    console.registerParamsForCommand("setGoToCommand", params);
    params.clear();

    console.registerCommand("setJointCommand", "send joint position command", [&](const std::vector<std::string> &params){
        robot_remote_control::JointCommand cmd;
        try {
            cmd.add_name(params[1]);
            if (params[0] == "position") {
                cmd.add_position(std::stof(params[2]));
            } else if (params[0] == "velocity") {
                cmd.add_velocity(std::stof(params[2]));
            } else if (params[0] == "effort") {
                cmd.add_effort(std::stof(params[2]));
            }
        } catch (const std::invalid_argument &e) {
            std::cout << "second value must be a number, was '" << params[1] <<"' " << std::endl;
            std::cout << e.what() << std::endl;
            return false;
        }
        cmd.PrintDebugString();
        controller.setJointCommand(cmd);
        return true;
    });
    params.push_back(ConsoleCommands::ParamDef("mode [position, velocity, effort]", "position"));
    params.back().defaultvalues.push_back(ConsoleCommands::DefaultParam("velocity"));
    params.back().defaultvalues.push_back(ConsoleCommands::DefaultParam("effort"));
    params.push_back(ConsoleCommands::ParamDef("joint_name (string)", "empty"));
    params.push_back(ConsoleCommands::ParamDef("position (float)", "0"));
    console.registerParamsForCommand("setJointCommand", params);
    params.clear();


    // defien lamda externally to be able to call it in init without output
    bool printRequestControllableJoints = false;
    auto requestControllableJoints = [&](const std::vector<std::string> &params) {
        robot_remote_control::JointState joints;
        if (controller.requestControllableJoints(&joints)) {
            if (printRequestControllableJoints) {
                joints.PrintDebugString();
            }
            // add param options to autocomplete
            for (auto &jointname : joints.name()) {
                console.addParamDefaultValue("setJointCommand", 1, jointname);
            }
            return true;
        } else {
            printf("no new data received \n");
            return false;
        }
    };
    console.registerCommand("requestControllableJoints", "print ControllableJoints set by the robot", requestControllableJoints);

    std::map<std::string, float> simpleActionNamesValues;
    console.registerCommand("setSimpleActionCommand", "execute a simpleaction", [&](const std::vector<std::string> &params){
        std::for_each(params.begin(), params.end(), [](const std::string &param) {
            std::cout << param << std::endl;
        });
        robot_remote_control::SimpleAction action;
        action.set_name(params[0]);
        try {
            action.set_state(std::stof(params[1]));
        } catch (const std::invalid_argument &e) {
            auto value = simpleActionNamesValues.find(params[1]);
            if (value != simpleActionNamesValues.end()) {
                action.set_state(value->second);
            } else {
                std::cout << "Simpleaction state must be a number, was '" << params[1] <<"' " << std::endl;
                std::cout << e.what() << std::endl;
                return false;
            }
        }
        controller.setSimpleActionCommand(action);
        return true;
    });
    params.push_back(ConsoleCommands::ParamDef("name (string)", "name"));
    params.push_back(ConsoleCommands::ParamDef("value (float)", "0"));
    console.registerParamsForCommand("setSimpleActionCommand", params);
    params.clear();


    bool printRequestSimpleActions = false;
    auto requestSimpleActions = [&](const std::vector<std::string> &params){
        robot_remote_control::SimpleActions actions;
        if (controller.requestSimpleActions(&actions)) {
            if (printRequestSimpleActions) {
                actions.PrintDebugString();
            }
            // add param options to autocomplete
            for (auto &simpleaction : actions.actions()) {
                console.addParamDefaultValue("setSimpleActionCommand", 0, simpleaction.name());

                for (auto namedvalue : simpleaction.type().value_names()) {
                    simpleActionNamesValues[namedvalue.name()] = namedvalue.value();
                    console.addParamDefaultValue("setSimpleActionCommand", 1, namedvalue.name(), "setSimpleActionCommand " + simpleaction.name());
                }
            }
            return true;
        } else {
            printf("no new data received \n");
            return false;
        }
    };

    console.registerCommand("requestSimpleActions", "request simple actions and add them to autocomplete", requestSimpleActions);

    robot_remote_control::Map map;
    console.registerCommand("requestMap", "get a map", [&](const std::vector<std::string> &params) {
        int id = std::atoi(params[0].c_str());
        if (controller.requestMap(&map, id)) {
            if (map.map().Is<robot_remote_control::BinaryMap>()) {
                robot_remote_control::BinaryMap bmap;
                map.map().UnpackTo(&bmap);
                printf("got map of type %s, size: %li bytes\n", bmap.type().c_str(), bmap.data().size());
            } else {
                printf("%s\n", map.ShortDebugString().c_str());
            }
            return true;
        } else {
            printf("no new data received \n");
            return false;
        }
    });
    params.push_back(ConsoleCommands::ParamDef("value (int)", "1"));
    console.registerParamsForCommand("requestMap", params);
    params.clear();


    /**
     * Special telemetry functions (non-default print etc.)
     */

    robot_remote_control::Image rrc_type_image;
    console.registerCommand("getImage", "get a single image and print its properties (without data)", [&](const std::vector<std::string> &params) {
        bool received = false;
        uint8_t channel = 0;
        if (params.size()) { \
            channel = std::atoi(params.front().c_str()); \
        }
        if (controller.getImage(&rrc_type_image, true, channel)) {
            printf("image type: %s (%ix%i) frame: %s size: %lu bytes\n", rrc_type_image.encoding().c_str(), rrc_type_image.width(), rrc_type_image.height(), rrc_type_image.header().frame().c_str(), rrc_type_image.data().size());
        } else {
            printf("no new data received on channel %i\n", channel);
        }
        return received;
    });
    params.push_back(ConsoleCommands::ParamDef("channel", "0", true));
    console.registerParamsForCommand("getImage", params);
    params.clear();


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

    console.registerCommand("requestRobotModel", "download a robot model", [&](const std::vector<std::string> &params) {
        std::pair<std::string, std::string> filename = controller.requestRobotModel(params[0]);
        if (filename.first != "" && filename.second != "") {
            printf("\ndownloaded model, please open '%s%s/%s' with an external viewer\n\n", params[0].c_str(), filename.first.c_str(), filename.second.c_str());
            return true;
        } else {
            printf("no model received, possibly the robot does not define one\n");
            return false;
        }
    });
    params.push_back(ConsoleCommands::ParamDef("target folder (string)", "./model"));
    console.registerParamsForCommand("requestRobotModel", params);
    params.clear();


    console.registerCommand("requestProtocolVersion", "print the protocol sha256sum",  [&](const std::vector<std::string> &params) {
        std::cout << "remote : " << controller.requestProtocolVersion() << std::endl;
        std::cout << "local  : " << controller.protocolVersion() << std::endl;
        return true;
    });

    console.registerCommand("requestLibraryVersion", "print the library version of the remote",  [&](const std::vector<std::string> &params) {
        std::cout << "remote : " << controller.requestLibraryVersion() << std::endl;
        std::cout << "local  : " << controller.libraryVersion() << std::endl;
        return true;
    });

    console.registerCommand("requestGitVersion", "print the git version of the remote",  [&](const std::vector<std::string> &params) {
        std::cout << "remote : " << controller.requestGitVersion() << std::endl;
        std::cout << "local  : " << controller.gitVersion() << std::endl;
        return true;
    });

    /**
     * Main loop
     */

    // set Heartbeat to one second, needed tp detect the connection
    controller.setHeartBeatDuration(1);

    controller.setupConnectedCallback([&](){
        printf("connected\n");
        console.redraw();
    });

    printf("waiting for connection\n");
    controller.waitForConnection();

    // call request lamda (output off)
    requestControllableJoints(std::vector<std::string>());
    printRequestControllableJoints = true;

    requestSimpleActions(std::vector<std::string>());
    printRequestSimpleActions = true;

    controller.checkProtocolVersion();
    controller.checkLibraryVersion();

    while (run)  {
        SUCCESS = console.readline("rrc@" + ip + " $ ", EXIT_ON_FAILURE);
        if (!SUCCESS) {
            break;
        }
    }

    printf("\n");
    fflush(stdout);
    controller.stopUpdateThread();
    return !SUCCESS;
}
