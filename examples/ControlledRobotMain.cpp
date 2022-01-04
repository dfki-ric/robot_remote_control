
#include "ControlledRobot.hpp"

#include <unistd.h>
#include <iostream>

#include "Transports/TransportZmq.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;


int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001", TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002", TransportZmq::PUB));
    robot_remote_control::ControlledRobot robot(commands, telemetry);

    robot.startUpdateThread(10);

    // set a callback for connection losses, allow 100ms of later arrival
    // (due to differences in latency between heartbeat commands)
    // the elapsed time may be used to have different stages of escalation
    robot.setupHeartbeatCallback(0.1, [](const float &elapsed){
        printf("no heartbeat since %.2f seconds\n", elapsed);
    });

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
    *transform.mutable_timestamp() = robot.getTime();
    *transforms.add_transform() = transform;

    robot.initSimpleActions(simpleActions);

    robot_remote_control::ComplexActions complexActions;
    robot.initComplexActions(complexActions);

    robot_remote_control::SimpleSensors sensors;
    robot_remote_control::SimpleSensor* sens;
    sens = sensors.add_sensors();
    sens->set_name("temperature");
    sens->set_id(1);
    sens->mutable_size()->set_x(1);  // only single value

    sens = sensors.add_sensors();
    sens->set_name("velocity");
    sens->set_id(2);
    sens->mutable_size()->set_x(3);  // 3 value vector

    robot.initSimpleSensors(sensors);

    // init done, now fake a robot

    // robot state
    robot_remote_control::JointState jointsstate = controllableJoints;
    robot_remote_control::Position position;
    robot_remote_control::Orientation orientation;
    robot_remote_control::Pose currentpose, targetpose;

    robot_remote_control::SimpleSensor temperature, velocity;
    temperature.set_id(1);
    // init value
    temperature.add_value(42);

    velocity.set_id(2);
    // init value
    velocity.add_value(0);
    velocity.add_value(0);
    velocity.add_value(0);

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



    // for requests to work, you need a valid connection:
    // only works when heartbeats are set up
    while (!robot.isConnected()) {
        printf("waiting for connection\n");
        usleep(100000);
    }

    robot.addCommandReceivedCallback(robot_remote_control::TARGET_POSE_COMMAND, []() {
        // WARNING: this callback run in the reveive thread, you should not use this to access data, only to notify other threads
        //printf("Pose Command Callback\n");
    });


    robot_remote_control::PermissionRequest permreq;
    permreq.set_description("test");
    permreq.set_requestuid("testuid");
    std::shared_future<bool> perm1 = robot.requestPermission(permreq);

    permreq.set_description("test2");
    permreq.set_requestuid("testuid2");
    std::shared_future<bool> perm2 = robot.requestPermission(permreq);

    //perm1.wait();
    //printf("result of permission request 1: %s\n", perm1.get() ? "true" : "false");

    while (true) {

        // if (perm2.valid()) {
        //     printf("result of permission request 2: %s\n", perm2.get() ? "true" : "false");
        // }

        // get and print commands
        if (robot.getTargetPoseCommand(&targetpose)) {
            printf("\ngot target pose command:\n%s\n", targetpose.ShortDebugString().c_str());
            robot.setLogMessage(robot_remote_control::INFO, "warping fake robot to target position\n");
            currentpose = targetpose;
            *(currentpose.mutable_timestamp()) = robot.getTime();
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

        if (robot.getSimpleActionCommand(&simpleactionscommand)) {
            printf("\ngot simple actions command:\n%s\n", simpleactionscommand.ShortDebugString().c_str());
            robot.setLogMessage(robot_remote_control::INFO, "setting simple action state\n");
            // do it
        }

        if (robot.getComplexActionCommand(&complexactionscommand)) {
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

        temperature.set_value(0, 42);
        robot.setSimpleSensor(temperature);

        robot.setCurrentTransforms(transforms);

        velocity.set_value(0, 0.1);
        velocity.set_value(1, 0.1);
        velocity.set_value(2, 0.1);
        robot.setSimpleSensor(velocity);

        
        robot_remote_control::Twist twistState;
        twistState.mutable_linear()->set_x(1);
        twistState.mutable_linear()->set_y(2);
        twistState.mutable_linear()->set_z(3);
        twistState.set_frame("Test for a framename");
        robot.setCurrentTwist(twistState);

        // when the define RRC_STATISTICS was active during compilation you can calculate/print/use stats
        // if not, the sats stay empty
        // robot.getStatistics().calculate();
        // robot.getStatistics().print(true);

        usleep(100000);
    }


    return 0;
}


