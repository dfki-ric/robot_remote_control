#include <iostream>
#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include <unistd.h>


using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;



//https://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
#include <stdio.h>
#include <sys/ioctl.h> // For FIONREAD
#include <termios.h>
int kbhit(void) {
    static bool initflag = false;
    static const int STDIN = 0;

    if (!initflag) {
        // Use termios to turn off line buffering
        struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initflag = true;
    }

    int nbbytes;
    ioctl(STDIN, FIONREAD, &nbbytes);  // 0 is STDIN
    return nbbytes;
}


int main(int argc, char** argv) {
    printf("Usage:\n");
    printf("up/down arrow: select joint\n");
    printf("left/right arrow: set position\n");
    printf("space: set position to 0\n");
    printf("+/-: select increment for position\n");

    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));
    robot_remote_control::RobotController controller(commands, telemetry);


    robot_remote_control::JointState jointstate;
    robot_remote_control::JointState contollablejoints;
    robot_remote_control::JointCommand jointcommand;


    controller.startUpdateThread(100);

    // set Heartbeat to one second
    controller.setHeartBeatDuration(1);

    sleep(2);  // give zmq time to connect
    controller.requestControllableJoints(&contollablejoints);
    contollablejoints.PrintDebugString();


    int joint_index = 0;

    float increment = 0.01;
    float value = 0;
    char input = 0;

    while (true) {
        int newstate = controller.getCurrentJointState(&jointstate);

        if (newstate) {
            // copy available to controllable joints
            jointcommand.Clear();
            for (int i = 0; i < jointstate.name().size(); ++i) {
                jointcommand.add_name(jointstate.name(i));
                if (jointstate.position().size()) {
                    jointcommand.add_position(jointstate.position(i));
                }
                if (jointstate.velocity().size()) {
                    jointcommand.add_velocity(jointstate.velocity(i));
                }
                if (jointstate.effort().size()) {
                    jointcommand.add_effort(jointstate.effort(i));
                }
                if (jointstate.acceleration().size()) {
                    jointcommand.add_acceleration(jointstate.acceleration(i));
                }
            }
            jointcommand.PrintDebugString();

            value = jointcommand.position(joint_index);
        }
            jointcommand.PrintDebugString();

        if (kbhit()) {
            printf("\r");  // carriage retun (overwrite input from key press)
            bool indexChagnged = false;
            input = getchar();
            //printf("%i\n", input);
            switch (input) {
                case 43:
                    increment *= 10;
                    printf("increment %.4f\n", increment);
                    break;
                case 45:
                    increment /= 10;
                    printf("increment %.4f\n", increment);
                    break;
                case 65:  // up
                    joint_index += 1;
                    indexChagnged = true;
                    break;
                case 66:  // down
                    joint_index -= 1;
                    indexChagnged = true;
                    break;
                case 67:  // right
                    value += increment;
                    break;
                case 68:  // left
                    value -= increment;
                    break;
                case 32:  // space
                    value = 0;
                    break;
                case 48:  // 0
                case 49:  // 1
                case 50:  // 2
                case 51:  // 3
                case 52:  // 4
                case 53:  // 5
                case 54:  // 6
                case 55:  // 7
                case 56:  // 8
                case 57:  // 9
                    joint_index = input-48;
                    indexChagnged = true;
                    break;
            }
            if (indexChagnged) {
                if (joint_index < 0) {
                    joint_index = 0;
                }
                if (joint_index >= jointstate.name_size()-1) {
                    joint_index = jointstate.name_size()-1;
                }
                value = jointstate.position(joint_index);
                indexChagnged = false;
            }
            printf("joint: %s value: %.2f\n", jointstate.name(joint_index).c_str(), value);

            // robot_remote_control::JointState jointcommand;
            // jointcommand.add_name(jointstate.name(joint_index).c_str());
            // jointcommand.add_position(value);

            jointcommand.set_position(joint_index, value);
            jointcommand.clear_effort();
            jointcommand.clear_velocity();
            controller.setJointCommand(jointcommand);

        } else {
            usleep(20000);
        }

        

        usleep(10000);
    }

    return 0;
}
