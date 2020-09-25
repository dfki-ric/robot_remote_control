#include <iostream>
#include "TransportUDT.hpp"
#include <unistd.h>

int main(int argc, char** argv)
{
    robot_remote_control::TransportUDT server(robot_remote_control::TransportUDT::SERVER,9000);
    sleep(4);

    for (unsigned int count = 0; count < 10; ++count){
        
        std::string msg = "hello world " + std::to_string(count);
	std::cout << "Sending: " << msg << std::endl;
        server.send(msg);
    
        sleep(3);
    }

    return 0;
}
