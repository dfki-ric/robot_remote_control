#include "TransportWebSocket.hpp"




using namespace std;
using namespace robot_remote_control;

TransportWebSocket::TransportWebSocket(const ConnectionType &type, const int &port, const std::string &addr) {
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    if (type == SERVER) {
        
        server = std::make_shared<WSServer>();

         // Set logging settings
        server->set_error_channels(websocketpp::log::elevel::all);
        server->set_access_channels(websocketpp::log::alevel::all ^ websocketpp::log::alevel::frame_payload);
 
        // Initialize Asio
        server->init_asio();

        server->set_open_handler([&](websocketpp::connection_hdl hdl){
            printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
            connections.insert(hdl);
        });

        server->set_close_handler([&](websocketpp::connection_hdl hdl){
            printf("server closing %s:%i\n", __PRETTY_FUNCTION__, __LINE__);
            connections.erase(hdl);
        });

        server->set_message_handler([&](websocketpp::connection_hdl hdl, WSServer::message_ptr msg_ptr) {
            printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
            recvQueueMutex.lock();
            recvQueue.push(msg_ptr->get_payload());
            recvQueueMutex.unlock();
        });

        thread.reset(new websocketpp::lib::thread([&](){
            getServer()->listen(port);
            // Queues a connection accept operation
            getServer()->start_accept();
            // Start the Asio io_service run loop
            getServer()->run();
        }));

    } else {

        client = std::make_shared<WSClient>();
        client->clear_access_channels(websocketpp::log::alevel::all);
        client->clear_error_channels(websocketpp::log::elevel::all);
        client->init_asio();
        client->start_perpetual();

        client->set_open_handler([&](websocketpp::connection_hdl hdl){
            printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
        });

        client->set_close_handler([&](websocketpp::connection_hdl hdl){
            printf("client closing %s:%i\n", __PRETTY_FUNCTION__, __LINE__);
        });

        websocketpp::lib::error_code ec;
        std::string uri = "ws://" + addr + ":" + std::to_string(port);
        printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__,uri.c_str());
        clientConnection = client->get_connection(uri, ec);

        client->connect(clientConnection);

        if (ec) {
            std::cout << "> Connect initialization error: " << ec.message() << std::endl;
        }

        thread.reset(new websocketpp::lib::thread([&](){
            getClient()->run();
        }));
    }

}

TransportWebSocket::~TransportWebSocket() {
    if (server){
        server->stop();
    }
}

int TransportWebSocket::send(const std::string& buf, Flags flags) {
    if (server) {
        for (auto listener : connections) {
            printf("%s:%i size: %li\n", __PRETTY_FUNCTION__, __LINE__, buf.size());
            server->send(listener, buf, websocketpp::frame::opcode::binary);
        }
    }else{
        printf("%s:%i size: %li\n", __PRETTY_FUNCTION__, __LINE__, buf.size());
        client->send(clientConnection, buf, websocketpp::frame::opcode::binary);
    }
    return buf.size();
}

int TransportWebSocket::receive(std::string* buf, Flags flags) {
    int res = 0;
    std::lock_guard<std::mutex> lock (recvQueueMutex);
    printf("%s:%i queue size: %li\n", __PRETTY_FUNCTION__, __LINE__, recvQueue.size());
    if (recvQueue.size()) {
        *buf = recvQueue.front();
        recvQueue.pop();
        
        printf("%s:%i size: %li\n", __PRETTY_FUNCTION__, __LINE__, buf->size());
        return buf->size();
    }
    return 0;
}
