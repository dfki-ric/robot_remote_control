#include "TransportZmq.hpp"
#include <zmq.hpp>

using namespace interaction;


TransportZmq::TransportZmq(const std::string &addr, const ConnectionType &type){
    
    context = std::shared_ptr<zmq::context_t>(new zmq::context_t(1));

    switch(type){
        case REQ:{
            socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_REQ));
            socket->connect(addr);
            break;
        }
        case REP:{
            socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_REP));
            socket->bind(addr);
            break;
        }
        case PUB:{
            socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_PUB));
            socket->bind(addr);
            break;
        }
        case SUB:{
            socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_SUB));
            socket->setsockopt(ZMQ_SUBSCRIBE, NULL, 0);//subscribe all
            socket->connect(addr);
            break;
        }
    }

}

int TransportZmq::send(const std::string& buf){
    zmq::message_t msg(buf.data(),buf.size());
    return socket->send(msg);
}

std::string TransportZmq::receive(){
    zmq::message_t requestmsg;
    socket->recv(&requestmsg);
    std::string msg ((char*)requestmsg.data(),requestmsg.size());
    return msg;
}

