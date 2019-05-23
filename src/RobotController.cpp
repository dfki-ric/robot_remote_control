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
    sendProtobufData(pose,TARGET_POSE);
}


int RobotController::setTwistCommand(const interaction::Twist &twistCommand)
{
    sendProtobufData(twistCommand,TWIST_COMMAND);
}


interaction::Pose RobotController::getCurrentPose()
{
    std::string buf;
    buf.resize(sizeof(uint16_t));
    uint16_t type = CURRENT_POSE;
    uint16_t* data = (uint16_t*)buf.data();
    *data = type;
    ControlMessageType received_type = evaluateReply(sendRequest(buf));
    if (received_type != CURRENT_POSE){
        printf("request returned wrong type of nothing\n");
        
    }
    return currentPose;
}

std::string RobotController::sendRequest(const std::string& serializedMessage){
    
    zmq::message_t msg(serializedMessage.data(),serializedMessage.size());
    socket->send(msg);
   
    zmq::message_t reply;
    socket->recv(&reply);

    std::string replystr ((char*)reply.data(),reply.size());

    return replystr;

    

}

ControlMessageType RobotController::evaluateReply(const std::string& reply){


    uint16_t* type = (uint16_t*)reply.data();

    std::string serializedMessage(reply.data()+sizeof(uint16_t),reply.size()-sizeof(uint16_t));

    ControlMessageType msgtype = (ControlMessageType)*type;

    printf("receive reply of type %i\n",msgtype);

    switch (msgtype){
        case CURRENT_POSE:{
            currentPose.ParseFromString(serializedMessage);
            return msgtype;
        } 
        
        default: return msgtype;
    }

    //should never reach this
    return NO_DATA;
}