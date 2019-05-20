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
    std::string buf;
    buf.resize(sizeof(uint16_t));
    uint16_t type = TARGET_POSE;
    uint16_t* data = (uint16_t*)buf.data();
    *data = type;
    pose.AppendToString(&buf);
    sendRequest(buf);

}


int RobotController::sendRequest(const std::string& serializedMessage){
    
    zmq::message_t msg(serializedMessage.data(),serializedMessage.size());
    socket->send(msg);
   
    zmq::message_t reply;
    socket->recv(&reply);

    std::string replystr ((char*)reply.data(),reply.size());

    int replytype =  evaluateReply(replystr);

    printf("receive reply of type %i\n",replytype);

    return replytype;


}

int RobotController::evaluateReply(const std::string& reply){


    uint16_t* type = (uint16_t*)reply.data();

    std::string serializedMessage(reply.data()+sizeof(uint16_t),reply.size()-sizeof(uint16_t));

    ControlMessageType msgtype = *((ControlMessageType*)type);

    switch (msgtype){
        case CURRENT_POSE:{
            currentPose.ParseFromString(serializedMessage);
        } 
        
        default: return msgtype;
    }

    //should never reach this
    return NO_DATA;
}