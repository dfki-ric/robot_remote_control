#include "TransportZmq.hpp"
#include <zmq.hpp>

using namespace robot_remote_control;

std::shared_ptr<zmq::context_t> TransportZmq::getContextInstance(unsigned int threads) {
    // implements a singleton using a local static variable
    static std::shared_ptr<zmq::context_t> contextInstance = std::shared_ptr<zmq::context_t>(new zmq::context_t(threads));
    return contextInstance;
}

TransportZmq::~TransportZmq() {
    disconnect();
}

TransportZmq::TransportZmq(const std::string &addr, const ConnectionType &type) : addr(addr), type(type), connected(false) {
    context = getContextInstance();
    connect();
}

int TransportZmq::send(const std::string& buf, Flags flags) {
    zmq::message_t msg(buf.data(), buf.size());

    zmq::send_flags zmqflag = zmq::send_flags::none;
    if (flags & NOBLOCK) {
        zmqflag = zmq::send_flags::dontwait;
    }
    if (socket->send(msg, zmqflag)) {
        return buf.size();
    }
    return 0;
}

int TransportZmq::receive(std::string* buf, Flags flags) {
    zmq::message_t requestmsg;
    zmq::recv_flags zmqflag = zmq::recv_flags::none;
    if (flags & NOBLOCK) {
        zmqflag = zmq::recv_flags::dontwait;
    }

    if (socket->recv(requestmsg, zmqflag)) {
        buf->assign(reinterpret_cast<char*>(requestmsg.data()), requestmsg.size());
    }

    return requestmsg.size();
}

void TransportZmq::connect() {
    if (!connected) {
        switch (type) {
            case REQ: {
                socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_REQ));

                #if CPPZMQ_VERSION < ZMQ_MAKE_VERSION(4, 7, 0)
                    socket->setsockopt(ZMQ_REQ_CORRELATE, 1);
                    socket->setsockopt(ZMQ_REQ_RELAXED, 1);
                #else
                    socket->set(zmq::sockopt::req_correlate, 1);
                    socket->set(zmq::sockopt::req_relaxed, 1);
                #endif

                socket->connect(addr);
                break;
            }
            case REP: {
                socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_REP));
                socket->bind(addr);
                break;
            }
            case PUB: {
                socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_PUB));
                socket->bind(addr);
                break;
            }
            case SUB: {
                socket = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(*(context.get()), ZMQ_SUB));

                #if CPPZMQ_VERSION < ZMQ_MAKE_VERSION(4, 7, 0)
                    socket->setsockopt(ZMQ_SUBSCRIBE, NULL, 0);  // subscribe all
                #else
                    socket->set(zmq::sockopt::subscribe, "");
                #endif
                
                socket->connect(addr);
                break;
            }
        }
        connected = true;
    }
}

void TransportZmq::disconnect() {
    if (connected && socket.get() && (type == REQ || type == SUB)) {
        socket->disconnect(addr);
        connected = false;
    }
}
