#include "RobotController.hpp"
#include <iostream>

using namespace std;
using namespace interaction;


RobotController::RobotController(const std::string &addr)
{
    context = std::shared_ptr<zmq::context_t>(new zmq::context_t(1));
    socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_REQ));
    
    socket->connect(addr);


}


int RobotController::setTargetPose(const interaction::Pose & pose)
{

}