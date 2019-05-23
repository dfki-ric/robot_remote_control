#include <iostream>
#include <interaction-library-controlled_robot/ControlledRobot.hpp>
#include <interaction-library-controlled_robot/Transports/TransportZmq.hpp>

using namespace interaction;

int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001",interaction::TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002",interaction::TransportZmq::PUB));
    ControlledRobot robot(commands,telemetry);
    

    while (true){
        robot.update();

        //fake-write requested pose to curretn pose
        robot.setCurrentPose(robot.getTargetPose());
    }


    return 0;
}
