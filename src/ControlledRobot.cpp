#include "ControlledRobot.hpp"
#include <iostream>

using namespace std;
using namespace interaction;


ControlledRobot::ControlledRobot(const std::string &addr)
{
    context = std::shared_ptr<zmq::context_t>(new zmq::context_t(1));
    socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_REP));
    
    socket->bind(addr);



}

void ControlledRobot::update()
{
    receiveRequest();
}

ControlMessageType ControlledRobot::receiveRequest()
{
    zmq::message_t requestmsg;
    socket->recv(&requestmsg);

    std::string msg ((char*)requestmsg.data(),requestmsg.size());

    ControlMessageType req = evaluateRequest(msg);

    printf("receive request of type %i\n",req);

    return req;
    
}

ControlMessageType ControlledRobot::evaluateRequest(const std::string& request)
{

    uint16_t* type = (uint16_t*)request.data();
    ControlMessageType msgtype = (ControlMessageType)*type;
    std::string serializedMessage(request.data()+sizeof(uint16_t),request.size()-sizeof(uint16_t));

    switch (msgtype){
        case CURRENT_POSE:{
            socket->send(serializeCurrentPose());
            return CURRENT_POSE;
        }
        case TARGET_POSE:{
            targetPose.ParseFromString(serializedMessage);
            socket->send(serializeControlMessageType(TARGET_POSE));
            return TARGET_POSE;
        }
        
        default: return msgtype;
    }

}

void ControlledRobot::addControlMessageType(std::string &buf, const ControlMessageType& type){
    int currsize = buf.size();
    buf.resize(currsize + sizeof(uint16_t));
    uint16_t typeint = type;
    uint16_t* data = (uint16_t*)(buf.data()+currsize);
    *data = type;
}

zmq::message_t ControlledRobot::serializeControlMessageType(const ControlMessageType& type){
    std::string buf;
    addControlMessageType(buf,type);
    return zmq::message_t(buf.data(),buf.size());
}

zmq::message_t ControlledRobot::serializeCurrentPose()
{
    std::string buf;
    addControlMessageType(buf,CURRENT_POSE);
    currentPose.AppendToString(&buf);
    zmq::message_t msg(buf.data(),buf.size());
    return msg;

}