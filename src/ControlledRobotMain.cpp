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
    robot.setRobotName(name);



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
    
    robot.setControllableJoints(controllableJoints);

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

    

    robot.setSimpleActions(simpleActions);

    ComplexActions complexActions;
    robot.setComplexActions(complexActions);


    //init done, now fake a robot


    JointState joints = controllableJoints;

  
    //consruct type
    Position position;
    Orientation orientation;
    Pose currentpose,targetpose;
    Twist twistcommand;




    //fill inital robot state
    position.set_x(0);
    position.set_y(0);
    position.set_z(0);

    orientation.set_x(0);
    orientation.set_y(0);
    orientation.set_z(0);
    orientation.set_w(1);
    
    *(currentpose.mutable_position()) = position;
    *(currentpose.mutable_orientation()) = orientation; 
    robot.setCurrentPose(currentpose);


    while (true){

        //get and print commands
        if (robot.getTargetPoseCommand(targetpose)){
            printf("received new target pose %.2f %.2f %.2f\n",targetpose.position().x(),targetpose.position().y(),targetpose.position().z());
        }
        
        if (robot.getTwistCommand(twistcommand)){
            printf("received new twist command %.2f %.2f %.2f\n",twistcommand.linear().x(),twistcommand.linear().y(),twistcommand.linear().z());
        }


        //set and send fake telemetry

        currentpose.mutable_position()->set_x(currentpose.mutable_position()->x() + 0.01);
        currentpose.mutable_position()->set_y(currentpose.mutable_position()->y() + 0.01);
        currentpose.mutable_position()->set_z(currentpose.mutable_position()->z() + 0.01);
        robot.setCurrentPose(currentpose);

        //fake some joint movement
        //joints.InitAsDefaultInstance
        robot.setJointState(joints);

        

        usleep(100000);
    }


    return 0;
}
