#pragma once 

#include "Transport.hpp"
#include <memory>
#include <string>


namespace zmq {
class context_t;
class socket_t;
}

namespace robot_remote_control {
class TransportZmq: public Transport {
 public:
    enum ConnectionType {REQ, REP, PUB, SUB};

    TransportZmq(const std::string &addr, const ConnectionType &type);
    virtual ~TransportZmq();

    static std::shared_ptr<zmq::context_t> getContextInstance(unsigned int threads = 1);

    int send(const std::string& buf, Flags flags = NONE);

    int receive(std::string* buf, Flags flags = NONE);

 private:
    std::string addr;
    ConnectionType type;
    std::shared_ptr<zmq::context_t> context;
    std::shared_ptr<zmq::socket_t> socket;
};
}  // namespace robot_remote_control
