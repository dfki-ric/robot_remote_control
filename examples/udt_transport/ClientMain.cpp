#include <iostream>
#include "TransportUDT.hpp"
#include <unistd.h>

int main(int argc, char** argv)
{
    robot_remote_control::TransportUDT client(robot_remote_control::TransportUDT::CLIENT,9000,"127.0.0.1");
    sleep(4);

    for (unsigned int count = 0; count < 10; count++){

        std::string buf;
        buf.resize(20);
        client.receive(&buf);
        std::cout << "recv: " << buf << std::endl;

        sleep(3);
    }
    
    return 0;
}
