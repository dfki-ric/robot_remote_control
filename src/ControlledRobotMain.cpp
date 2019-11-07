#include <iostream>
#include "ControlledRobot.hpp"
#include "Transports/TransportZmq.hpp"
#include <unistd.h>

using namespace robot_remote_control;

int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001",TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002",TransportZmq::PUB));
    ControlledRobot robot(commands,telemetry);

    robot.startUpdateThread(100);    

    RobotName name;
    name.set_value("TestRobot");
    robot.initRobotName(name);
    robot.setRobotState("INIT");

    VideoStreams streams;
    VideoStream* stream = streams.add_stream();
    stream->set_url("http://robot/stream");
    robot.initVideoStreams(streams);

    JointState controllableJoints;
    //add a position controlled joint
    controllableJoints.add_name("testJoint1");
    controllableJoints.add_position(1);
    controllableJoints.add_velocity(0);
    controllableJoints.add_effort(0);

    //add a position velocity joint
    controllableJoints.add_name("testJoint2");
    controllableJoints.add_position(0);
    controllableJoints.add_velocity(1);
    controllableJoints.add_effort(0);
    
    robot.initControllableJoints(controllableJoints);

    SimpleActions simpleActions;
    // SimpleAction action;

    SimpleAction* action;
    action = simpleActions.add_actions();
    action->set_name("EmergencyStop");
    action->mutable_type()->set_type(SimpleActionType::TRIGGER);
    action->set_state(0); //set current state
    
    action = simpleActions.add_actions();
    action->set_name("Light");
    action->mutable_type()->set_type(SimpleActionType::VALUE);
    action->mutable_type()->set_max_state(1); //is just a switch on/off
    action->set_state(1); //current state

    action = simpleActions.add_actions();
    action->set_name("Light Dimmer");
    action->mutable_type()->set_type(SimpleActionType::VALUE);
    action->mutable_type()->set_max_state(100); //values from 0-100
    action->set_state(100); //current state

    

    robot.initSimpleActions(simpleActions);

    ComplexActions complexActions;
    //TODO fill data
    robot.initComplexActions(complexActions);


    SimpleSensors sensors;
    SimpleSensor* sens;
    sens = sensors.add_sensors();
    sens->set_name("temperature");
    sens->set_id(1);
    sens->mutable_size()->set_x(1); //only single value

    sens = sensors.add_sensors();
    sens->set_name("velocity");
    sens->set_id(2);
    sens->mutable_size()->set_x(3); // 3 value vector

    robot.initSimpleSensors(sensors);

    //init done, now fake a robot

    //robot state
    JointState jointsstate = controllableJoints;
    Position position;
    Orientation orientation;
    Pose currentpose,targetpose;

    SimpleSensor temperature,velocity;
    temperature.set_id(1);
    //init value
    temperature.add_value(42);

    velocity.set_id(2);
    //init value
    velocity.add_value(0);
    velocity.add_value(0);
    velocity.add_value(0);
    
    //commands
    Twist twistcommand;
    GoTo gotocommand;
    JointState jointscommand;
    SimpleActions simpleactionscommand;
    ComplexActions complexactionscommand;





    //fill inital robot state
    currentpose.mutable_position();
    currentpose.mutable_orientation()->set_w(1);
    robot.setCurrentPose(currentpose);


    while (true){

        //get and print commands
        if (robot.getTargetPoseCommand(targetpose)){
            printf("\ngot target pose command:\n%s\n",targetpose.ShortDebugString().c_str());
            robot.setLogMessage(INFO, "warping fake robot to target position\n");
            currentpose = targetpose;
        }
        
        if (robot.getTwistCommand(twistcommand)){
            printf("\ngot twist command:\n%s\n",twistcommand.ShortDebugString().c_str());
            robot.setLogMessage(ERROR, "twist not supported for fake robot\n");
        }

        if (robot.getGoToCommand(gotocommand)){
            printf("\ngot goto command:\n%s\n",gotocommand.ShortDebugString().c_str());
            robot.setLogMessage(ERROR, "goto not supported for fake robot\n");
        }

        if (robot.getJointsCommand(jointscommand)){
            printf("\ngot joints command:\n%s\n",jointscommand.ShortDebugString().c_str());
            robot.setLogMessage(INFO, "setting fake robot joints to target position\n");
            jointsstate = jointscommand;
        }

        if (robot.getSimpleActionsCommand(simpleactionscommand)){
            printf("\ngot simple actions command:\n%s\n",simpleactionscommand.ShortDebugString().c_str());
            robot.setLogMessage(INFO, "setting simple action state\n");
            //do it
        }

        if (robot.getComplexActionsCommand(complexactionscommand)){
            printf("\ngot complex actions command:\n%s\n",complexactionscommand.ShortDebugString().c_str());
            robot.setLogMessage(INFO, "setting complex action state\n");
            //do it
        }

        //set state
        robot.setRobotState("RUNNING");

        // set/send and send fake telemetry
        robot.setCurrentPose(currentpose);

        // fake some joint movement
        robot.setJointState(jointsstate);

        temperature.set_value(0,42);
        robot.setSimpleSensor(temperature);

        velocity.set_value(0,0.1);
        velocity.set_value(1,0.1);
        velocity.set_value(2,0.1);
        robot.setSimpleSensor(velocity);
        

        usleep(100000);
    }


    return 0;
}
