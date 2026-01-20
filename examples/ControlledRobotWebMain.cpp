
#include "ControlledRobot.hpp"

#include <memory>
#include <unistd.h>
#include <iostream>

#include "Transports/ZeroMQ/TransportZmq.hpp" // in a non-example use #include <robot_remote_control/Transports/TransportZmq>
#include "Transports/WebSocket/TransportWebSocket.hpp"
#include "Transports/Http/TransportHttp.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;
using robot_remote_control::TransportWebSocket;
using robot_remote_control::TransportHttp;

int main(int argc, char** argv)
{
    
    // TransportWebSocket may run in SERVER or SERVER_TEXT mode, SERVER_TEXT tells the ControlledRobot, that binary can't be transmitted
    // als switches to JSON serialization, this is required to connect the WS from a browser, if just SERVER is set, data will be binary
    //TransportSharedPtr commands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::SERVER_TEXT, 7001));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::SERVER_TEXT, 7002));
    TransportSharedPtr commands = TransportSharedPtr(new TransportHttp("http://0.0.0.0:7001"));

    std::dynamic_pointer_cast<TransportHttp>(commands)->serveFolder("examples/html");


    robot_remote_control::ControlledRobot robot(commands,telemetry);

    robot.startUpdateThread(10);

    // set a callback for connection losses, allow 100ms of later arrival
    // (due to differences in latency between heartbeat commands)
    // the elapsed time may be used to have different stages of escalation
    // when there are multiple connections to this robots with different heartbeats
    // in rare occations the logner heartbeat is used (connection loss (hight freq) right after the low freq time was send)
    robot.setupDisconnectedCallback(0.2, [](const float &elapsed){
        printf("no heartbeat since %.2f seconds\n", elapsed);
    });

    robot.setupConnectedCallback([](){
        printf("controller connected\n");
    });

    robot.setupHeartbeatCallbackInterval(2);

    robot_remote_control::RobotName name;
    name.set_value("TestRobot");
    robot.initRobotName(name);
    robot.setRobotState("INIT");

    robot_remote_control::VideoStreams streams;
    robot_remote_control::VideoStream* stream = streams.add_stream();
    stream->set_url("http://robot/stream");
    robot.initVideoStreams(streams);

    robot_remote_control::JointState controllableJoints;
    // add a position controlled joint
    controllableJoints.add_name("testJoint1");
    controllableJoints.add_position(1);
    controllableJoints.add_velocity(0);
    controllableJoints.add_effort(0);

    // add a position velocity joint
    controllableJoints.add_name("testJoint2");
    controllableJoints.add_position(0);
    controllableJoints.add_velocity(1);
    controllableJoints.add_effort(0);
    robot.initControllableJoints(controllableJoints);

    robot_remote_control::SimpleActions simpleActions;
    // SimpleAction action;

    robot_remote_control::SimpleAction* action;
    action = simpleActions.add_actions();
    action->set_name("EmergencyStop");
    action->mutable_type()->set_type(robot_remote_control::SimpleActionType::TRIGGER);
    action->set_state(0);  // set current state

    action = simpleActions.add_actions();
    action->set_name("Light");
    action->mutable_type()->set_type(robot_remote_control::SimpleActionType::VALUE_INT);
    action->mutable_type()->set_min_state(0);  // is just a switch on/off
    action->mutable_type()->set_max_state(1);  // is just a switch on/off
    action->set_state(1);  // current state

    action = simpleActions.add_actions();
    action->set_name("Light Dimmer");
    action->mutable_type()->set_type(robot_remote_control::SimpleActionType::VALUE_INT);
    action->mutable_type()->set_min_state(0);  // values from 0-100
    action->mutable_type()->set_max_state(100);  // values from 0-100
    robot_remote_control::ActionDependency * actiondep = action->mutable_type()->add_action_dependency();
    actiondep->set_depends_on_action("Light");
    actiondep->set_depends_on_action_in_state(1);

    action->set_state(100);  // current state



    // Transform
    robot_remote_control::Transforms transforms;
    robot_remote_control::Transform transform;
    robot_remote_control::Pose tf_pose;
    tf_pose.mutable_position()->set_x(7);
    tf_pose.mutable_position()->set_y(3);
    *transform.mutable_transform() = tf_pose;
    transform.set_from("Source");
    transform.set_to("Target");
    *transform.mutable_header()->mutable_timestamp() = robot.getTime();
    *transforms.add_transform() = transform;

    robot.initSimpleActions(simpleActions);

    robot_remote_control::ComplexActions complexActions;
    robot.initComplexActions(complexActions);

    // init done, now fake a robot

    // robot state
    robot_remote_control::JointState jointsstate = controllableJoints;
    robot_remote_control::Position position;
    robot_remote_control::Orientation orientation;
    robot_remote_control::Pose currentpose, targetpose;

    robot_remote_control::SimpleSensor temperature, velocity;
    // init value
    temperature.add_value(42);

    // send velocity in a "channel" (seperate buffers in the RobotController)
    // channels are numbered uint8_t, default channel is 0, first channel added is 1, etc.)
    int velocity_channel_no = robot.addChannel(robot_remote_control::SIMPLE_SENSOR, "velocity");

    // init value
    velocity.add_value(0);
    velocity.add_value(0);
    velocity.add_value(0);



    robot_remote_control::FileDefinition files;
    robot_remote_control::File* file;

    file = files.add_file();
    files.add_isfolder(true);
    file->set_identifier("folder");
    file->set_path("./test/testfiles/");

    file = files.add_file();
    files.add_isfolder(false);
    file->set_identifier("topfolderfile");
    file->set_path("./test/testfiles/topfolderfile");

    file = files.add_file();
    files.add_isfolder(false);
    file->set_identifier("subfolderfile");
    file->set_path("./test/testfiles/subfolder/subfolderfile");

    robot.initFiles(files);




    // commands
    robot_remote_control::Twist twistcommand;
    robot_remote_control::GoTo gotocommand;
    robot_remote_control::JointCommand jointscommand;
    robot_remote_control::SimpleAction simpleactionscommand;
    robot_remote_control::ComplexAction complexactionscommand;
    robot_remote_control::Poses posescommand;


    // fill inital robot state
    currentpose.mutable_position();
    currentpose.mutable_orientation()->set_w(1);
    robot.setCurrentPose(currentpose);

    // define an extra channel to send two poses (only needed when you need extra receive buffers on the RobotController)
    // otherwise use the header.frame field instead of headers to differentiate poses
    uint8_t pose2channelno = robot.addChannel(robot_remote_control::CURRENT_POSE, "2nd pose via channel");

    // for requests to work, you need a valid connection:
    // only works when heartbeats are set up
    // while (!robot.isConnected()) {
    //     printf("waiting for connection\n");
    //     usleep(100000);
    // }

    robot.addCommandReceivedCallback(robot_remote_control::TARGET_POSE_COMMAND, []() {
        // WARNING: this callback run in the reveive thread, you should not use this to access data, only to notify other threads
        // printf("Pose Command Callback\n");
    });


    robot_remote_control::PermissionRequest permreq;
    permreq.set_description("test");
    permreq.set_requestuid("testuid");
    std::shared_future<bool> perm1 = robot.requestPermission(permreq);

    permreq.set_description("test2");
    permreq.set_requestuid("testuid2");
    std::shared_future<bool> perm2 = robot.requestPermission(permreq);

    // perm1.wait();
    // printf("result of permission request 1: %s\n", perm1.get() ? "true" : "false");

    while (true) {
        // if (perm2.valid()) {
        //     printf("result of permission request 2: %s\n", perm2.get() ? "true" : "false");
        // }

        // get and print commands
        if (robot.getTargetPoseCommand(&targetpose)) {
            printf("\ngot target pose command:\n%s\n", targetpose.ShortDebugString().c_str());
            robot.setLogMessage(robot_remote_control::INFO, "warping fake robot to target position\n");
            currentpose = targetpose;
            *(currentpose.mutable_header()->mutable_timestamp()) = robot.getTime();
        }

        if (robot.getTwistCommand(&twistcommand)) {
            printf("\ngot twist command:\n%s\n", twistcommand.ShortDebugString().c_str());
            robot.setLogMessage(robot_remote_control::ERROR, "twist not supported for fake robot\n");
        }

        if (robot.getGoToCommand(&gotocommand)) {
            printf("\ngot goto command:\n%s\n", gotocommand.ShortDebugString().c_str());
            robot.setLogMessage(robot_remote_control::ERROR, "goto not supported for fake robot\n");
        }

        if (robot.getJointsCommand(&jointscommand)) {
            printf("\ngot joints command:\n%s\n", jointscommand.ShortDebugString().c_str());
            robot.setLogMessage(robot_remote_control::INFO, "setting fake robot joints to target position\n");
            jointsstate.Clear();
            for (int i = 0; i < jointscommand.name().size(); ++i) {
                jointsstate.add_name(jointscommand.name(i));
                if (jointscommand.position().size()) {
                    jointsstate.add_position(jointscommand.position(i));
                }
                if (jointscommand.velocity().size()) {
                    jointsstate.add_velocity(jointscommand.velocity(i));
                }
                if (jointscommand.effort().size()) {
                    jointsstate.add_effort(jointscommand.effort(i));
                }
                if (jointscommand.acceleration().size()) {
                    jointsstate.add_acceleration(jointscommand.acceleration(i));
                }
            }
        }

        while (robot.getSimpleActionCommand(&simpleactionscommand)) {
            printf("\ngot simple actions command:\n%s\n", simpleactionscommand.ShortDebugString().c_str());
            robot.setLogMessage(robot_remote_control::INFO, "setting simple action state\n");
            // do it
        }

        while (robot.getComplexActionCommand(&complexactionscommand)) {
            printf("\ngot complex actions command:\n%s\n", complexactionscommand.ShortDebugString().c_str());
            robot.setLogMessage(robot_remote_control::INFO, "setting complex action state\n");
            // do it
        }

        if (robot.getRobotTrajectoryCommand(&posescommand)) {
            printf("\ngot trajectory command:\n%s\n", posescommand.ShortDebugString().c_str());
        }

        // set state
        robot.setRobotState("RUNNING");

        // set/send and send fake telemetry
        robot.setCurrentPose(currentpose);

        // fake some joint movement
        robot.setJointState(jointsstate);

        robot.setCurrentTransforms(transforms);

        temperature.set_value(0, 42);
        robot.setSimpleSensor(temperature);

        velocity.set_value(0, 0.1);
        velocity.set_value(1, 0.1);
        velocity.set_value(2, 0.1);
        robot.setSimpleSensor(velocity, velocity_channel_no);  // 2nd param is the channelno.


        robot_remote_control::Twist twistState;
        twistState.mutable_linear()->set_x(1);
        twistState.mutable_linear()->set_y(2);
        twistState.mutable_linear()->set_z(3);
        twistState.mutable_header()->set_frame("Test for a framename");
        robot.setCurrentTwist(twistState);

        // when the define RRC_STATISTICS was active during compilation you can calculate/print/use stats
        // if not, the sats stay empty
        // robot.getStatistics().calculate();
        // robot.getStatistics().print(true);

        usleep(100000);
    }


    return 0;
}


