#include <iostream>
#include <interaction-library-controlled_robot/ControlledRobot.hpp>
#include <interaction-library-controlled_robot/Transports/TransportZmq.hpp>

int main(int argc, char** argv)
{
    std::shared_ptr<interaction::Transport> commands = std::shared_ptr<interaction::Transport>(new interaction::TransportZmq("tcp://*:7001",interaction::TransportZmq::REP));
    std::shared_ptr<interaction::Transport> telemetry = std::shared_ptr<interaction::Transport>(new interaction::TransportZmq("tcp://*:7002",interaction::TransportZmq::PUB));
    interaction::ControlledRobot robot(commands,telemetry);
    

    while (true){
        robot.update();

        //fake-write requested pose to curretn pose
        robot.setCurrentPose(robot.getTargetPose());
    }


    return 0;
}
