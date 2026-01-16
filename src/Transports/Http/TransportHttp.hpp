# pragma once

#include <future>
#include <utility>

#include <cpprest/http_client.h>

#include "Transport.hpp"
#include "RestServer.hpp"

#include "../Serialization.hpp"
#include "../UpdateThread/LockableClass.hpp"

namespace robot_remote_control {

class TransportHttp : public Transport {
 public:

    enum ConnectionType {SERVER,CLIENT};

    TransportHttp(const std::string& url, const ConnectionType& mode = SERVER);
    virtual ~TransportHttp();

    /**
     * @brief TODO: allow serverFolder
     * 
     */

    /**
     * @brief send data
     * 
     * @param buf the buffer to send
     * @param Flags flags the flags
     * @return int number of bytes sent
     */
    virtual int send(const std::string& buf, Flags flags = NONE);

    /**
     * @brief receive data
     * 
     * @param buf buffer to fill on receive
     * @param Flags flags the flags
     * @return int 0 if no data received, size of data otherwise
     */
    virtual int receive(std::string* buf, Flags flags = NONE);

    /**
     * @brief some transports may require a non-binary format
     * 
     * @return true 
     * @return false 
     */
    virtual bool requiresTextProtocol() {
        return true;
    };

    virtual bool requiresRobotControllerPointer() {
        return true;
    }

 private:

    std::future<std::string> requestTelemetry(const TelemetryMessageType type, const int &channel = 0);

    Serialization serialization;
    struct Request {
        std::string request;
        std::shared_ptr<std::promise<std::string>> reply;
    };

    LockableClass<std::queue<Request>> recvQueue;
    
    LockableClass<std::queue<std::string>> clientRecvQueue;

    Request activeRequest;
    std::unique_ptr<rest_api::RestServer> server;
    std::unique_ptr<web::http::client::http_client> client;

};

}  // namespace robot_remote_control
