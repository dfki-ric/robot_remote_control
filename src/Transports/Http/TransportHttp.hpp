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

    enum ConnectionType {SERVER, CLIENT};

    /**
     * @brief Construct a new Transport Http object
     * 
     * @param url follows the https://github.com/microsoft/cpprestsdk syntax, e.g. http://0.0.0.0:7001 listen on all ips on port 7001
     * @param mode can be server and client
     */
    TransportHttp(const std::string& url, const ConnectionType& mode = SERVER);
    virtual ~TransportHttp();

    /**
     * @brief TODO: allow serverFolder
     * 
     */
    void serveFolder(const std::string& folderpath, const std::string& url = "html");

    /**
     * @brief Set the Default Page if unset, a default listing (api/html folder) is shown
     * 
     * @param localurl local file like "html/index.html"
     */
    void setDefaultPage(const std::string& localurl);

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

 private:

    void setSerializationMode(const Serialization::Mode & mode) override {}

    TelemetryMessageType getTelemetryType(const std::string &param);
    ControlMessageType getControlType(const std::string &param);

    template <class PROTO> std::string getDocJsonString(const PROTO &proto) {
        std::string json;
        docSerialization.serialize(proto, &json);
        return json;
    }

    std::future<std::string> requestTelemetry(const TelemetryMessageType type, const int &channel = 0);

    // Serialization serialization;
    Serialization docSerialization;
    Serialization sampleSerialization;

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
