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
    

    while (true){
        robot.update(); //non blocking

        //fake-write requested pose to curretn pose
        robot.setCurrentPose(robot.getTargetPose());

        usleep(100000);
    }


    return 0;
}
