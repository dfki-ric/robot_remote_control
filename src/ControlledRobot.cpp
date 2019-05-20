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


int ControlledRobot::receiveRequest()
{
    zmq::message_t requestmsg;
    socket->recv(&requestmsg);

    std::string msg ((char*)requestmsg.data(),requestmsg.size());

    int req = evaluateRequest(msg);

    printf("receive request of type %i\n",req);

    return req;
    
}

int ControlledRobot::evaluateRequest(const std::string& request)
{

    uint16_t* type = (uint16_t*)request.data();
    ControlMessageType msgtype = *((ControlMessageType*)type);
    std::string serializedMessage(request.data()+sizeof(uint16_t),request.size()-sizeof(uint16_t));

    switch (msgtype){
        case CURRENT_POSE:{
            socket->send(serializeCurrentPose());
        }
        case TARGET_POSE:{
            targetPose.ParseFromString(serializedMessage);
        }
        
        default: return msgtype;
    }

}


zmq::message_t ControlledRobot::serializeCurrentPose()
{
    std::string buf;
    currentPose.SerializeToString(&buf);
    zmq::message_t msg(buf.data(),buf.size());
    return msg;

}