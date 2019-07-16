#include <iostream>
#include <interaction-library-controlled_robot/ControlledRobot.hpp>
#include <interaction-library-controlled_robot/Transports/TransportZmq.hpp>
#include <unistd.h>

using namespace controlledRobot;

int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001",TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002",TransportZmq::PUB));
    ControlledRobot robot(commands,telemetry);

    robot.startUpdateThread(100);    

    while (true){
        //robot.update(); //blocking

        Pose pose = robot.getTargetPose();
        //fake-write requested pose to curretn pose
        robot.setCurrentPose(pose);

        printf("%.2f %.2f %.2f\n",pose.position().x(),pose.position().y(),pose.position().z());

        usleep(100000);
    }


    return 0;
}
