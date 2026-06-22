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

    /**
     * @brief init the ZMQ context with a number of threads
     * The context is a singleton, that is shared amont all instances of TransportZmq
     * 
     * @warning This methond has to be called before instaciating any TransportZmq object
     * 
     * @param threads number of zmq context threads
     */
    static void initContextThreads(unsigned int threads) {
        getContextInstance(threads);
    }

    /**
     * @brief Get the Context Instance object
     * 
     * @param threads 
     * @return std::shared_ptr<zmq::context_t> 
     */
    static std::shared_ptr<zmq::context_t> getContextInstance(unsigned int threads = 2);

    int send(const std::string& buf, Flags flags = NONE);

    int receive(std::string* buf, Flags flags = NONE);

    /**
     * @brief disconnect socket (only for connection tests)
     */
    void disconnect();
    void connect();

 private:
    bool connected;
    std::string addr;
    ConnectionType type;
    std::shared_ptr<zmq::context_t> context;
    std::shared_ptr<zmq::socket_t> socket;
};
}  // namespace robot_remote_control
