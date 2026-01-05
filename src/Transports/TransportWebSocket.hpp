#pragma once

#include "Transport.hpp"

#include <memory>
#include <set>
#include <queue>

#include <websocketpp/server.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/config/asio_no_tls.hpp>

namespace robot_remote_control {


class TransportWebSocket : public Transport {
 public: 
    typedef websocketpp::client<websocketpp::config::asio_client> WSClient;
    typedef websocketpp::server<websocketpp::config::asio> WSServer;
    typedef std::set<websocketpp::connection_hdl,std::owner_less<websocketpp::connection_hdl>> Connections;
    
    enum ConnectionType {SERVER,CLIENT};

    TransportWebSocket(const ConnectionType &type, const int &port, const std::string &addr = "");
    virtual ~TransportWebSocket();

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

    std::shared_ptr<WSClient> getClient() {
        return client;
    }
    std::shared_ptr<WSServer> getServer() {
        return server;
    }

 private:
    std::shared_ptr<WSClient> client;
    std::shared_ptr<WSServer> server;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> thread;
    Connections connections;
    
    WSClient::connection_ptr clientConnection;
    std::queue<std::string> recvQueue;
    std::mutex recvQueueMutex;
};

} // end namespace robot_remote_control
