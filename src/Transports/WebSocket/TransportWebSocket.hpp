#pragma once

#include "Transport.hpp"

#include <map>
#include <memory>
#include <set>
#include <queue>

#include <websocketpp/server.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/config/asio_no_tls.hpp>

#include "../UpdateThread/LockableClass.hpp"

namespace robot_remote_control {


class TransportWebSocket : public Transport {
 public: 
    typedef websocketpp::client<websocketpp::config::asio_client> WSClient;
    typedef websocketpp::server<websocketpp::config::asio> WSServer;
    typedef std::set<websocketpp::connection_hdl,std::owner_less<websocketpp::connection_hdl>> Connections;
    // typedef std::vector<websocketpp::connection_hdl> Connections;
    
    enum ConnectionType {SERVER,CLIENT,SERVER_TEXT,CLIENT_TEXT};

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

    bool receiveMessage(std::string* buf, Flags flags = NONE) override;

    std::shared_ptr<WSClient> getClient() {
        return client;
    }
    std::shared_ptr<WSServer> getServer() {
        return server;
    }

    void setSerializationMode(const Serialization::Mode & mode) override;

    /**
     * @brief serve the files of a local folder via http on the websocket port (SERVER and SERVER_TEXT only),
     * e.g. a browser interface that connects to the websocket: http://host:port/html/index.html
     * The root url lists the served folders, folders show a directory listing.
     *
     * @param folderpath the local folder to serve
     * @param url the first part of the url path the folder is served at
     */
    void serveFolder(const std::string& folderpath, const std::string& url = "html");

 private:
    void handleHttpRequest(websocketpp::connection_hdl hdl);

    // url -> local folder
    LockableClass<std::map<std::string, std::string>> servedFolders;

    std::shared_ptr<WSClient> client;
    std::shared_ptr<WSServer> server;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> thread;
    
    LockableClass<Connections> connections;
    LockableClass<std::queue<std::string>> recvQueue;
    
    WSClient::connection_ptr clientConnection;
    bool clientConnected;

    websocketpp::frame::opcode::value opcode;

};

} // end namespace robot_remote_control
