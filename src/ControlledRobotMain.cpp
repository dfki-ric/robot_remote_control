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
    //TODO: set some values
    robot.setControllableJoints(controllableJoints);

    SimpleActions simpleActions;
    // SimpleAction action;

    SimpleAction* action;
    action = simpleActions.add_actions();
    action->set_name("EmergencyStop");
    action->set_state(1);
    
    action = simpleActions.add_actions();
    action->set_name("Light");
    action->set_state(0);
    

    robot.setSimpleActions(simpleActions);

    ComplexActions complexActions;
    robot.setComplexActions(complexActions);


    JointState joints = controllableJoints;


  
    //consruct type
    Position position;
    Orientation orientation;
    Pose pose;

    //fill
    position.set_x(0);
    position.set_y(0);
    position.set_z(0);

    orientation.set_x(0);
    orientation.set_y(0);
    orientation.set_z(0);
    orientation.set_w(1);
    
    *(pose.mutable_position()) = position;
    *(pose.mutable_orientation()) = orientation; 
    robot.setCurrentPose(pose);


    while (true){
        //robot.update(); //blocking

        pose = robot.getTargetPose();
        
        //fake-write requested pose to curretn pose
        robot.setCurrentPose(pose);

        //fake some joint movement
        robot.setJointState(joints);

        printf("%.2f %.2f %.2f\n",pose.position().x(),pose.position().y(),pose.position().z());

        usleep(100000);
    }


    return 0;
}
