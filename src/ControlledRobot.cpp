#include "ControlledRobot.hpp"
#include <iostream>

using namespace std;
using namespace interaction;


ControlledRobot::ControlledRobot(const std::string &addr)
{
    context = std::shared_ptr<zmq::context_t>(new zmq::context_t(1));
    socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_REP));
    
    socket->connect(addr);


}
