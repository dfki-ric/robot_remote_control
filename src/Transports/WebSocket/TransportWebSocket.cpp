#include "TransportWebSocket.hpp"

using namespace std;
using namespace robot_remote_control;

//https://github.com/zaphoyd/websocketpp/blob/master/examples/telemetry_server/telemetry_server.cpp

TransportWebSocket::TransportWebSocket(const ConnectionType &type, const int &port, const std::string &addr):clientConnected(false) {

    if (type == SERVER_TEXT || type == CLIENT_TEXT) {
        opcode = websocketpp::frame::opcode::text;
    }else{
        opcode = websocketpp::frame::opcode::binary;
    }

    if (type == SERVER || type == SERVER_TEXT) {
        setTransportSupport(ROBOTCOMMANDS | ROBOTTELEMETRY);
        server = std::make_shared<WSServer>();

         // Set logging settings
        // server->set_error_channels(websocketpp::log::elevel::all);
        // server->set_access_channels(websocketpp::log::alevel::all ^ websocketpp::log::alevel::frame_payload);
        server->clear_access_channels(websocketpp::log::alevel::all);
        server->clear_error_channels(websocketpp::log::elevel::all);
 
        // Initialize Asio
        server->init_asio();

        server->set_reuse_addr(true);

        server->set_open_handler([&](websocketpp::connection_hdl hdl){
            connections.lockedAccess()->insert(hdl);
        });

        server->set_close_handler([&](websocketpp::connection_hdl hdl){
            connections.lockedAccess()->erase(hdl);
        });

        server->set_message_handler([&](websocketpp::connection_hdl hdl, WSServer::message_ptr msg_ptr) {
            recvQueue.lockedAccess()->push(msg_ptr->get_payload());
        });

        // server->set_open_handler(std::bind(&TransportWebSocket::on_server_open, this, std::placeholders::_1));
        // server->set_close_handler(std::bind(&TransportWebSocket::on_server_close, this, std::placeholders::_1));
        // server->set_message_handler(std::bind(&TransportWebSocket::on_server_message, this, std::placeholders::_1, std::placeholders::_2));

        thread.reset(new websocketpp::lib::thread([this, port](){
            this->getServer()->listen(port);
            // Queues a connection accept operation
            this->getServer()->start_accept();
            // Start the Asio io_service run loop
            this->getServer()->run();
        }));

    } else {
        setTransportSupport(CONTOLLERCOMMANDS | CONTROLLERTELEMETRY);
        client = std::make_shared<WSClient>();
        
        // server->set_error_channels(websocketpp::log::elevel::all);
        // server->set_access_channels(websocketpp::log::alevel::all ^ websocketpp::log::alevel::frame_payload);
        client->clear_access_channels(websocketpp::log::alevel::all);
        client->clear_error_channels(websocketpp::log::elevel::all);

        client->init_asio();
        client->start_perpetual();

        client->set_open_handler([&](websocketpp::connection_hdl hdl){
            clientConnected = true;
        });

        client->set_close_handler([&](websocketpp::connection_hdl hdl){
            clientConnected = false;
        });

        client->set_message_handler([&](websocketpp::connection_hdl hdl, WSServer::message_ptr msg_ptr) {
            recvQueue.lockedAccess()->push(msg_ptr->get_payload());
        });

        websocketpp::lib::error_code ec;
        std::string uri = "ws://" + addr + ":" + std::to_string(port);
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
    if (client){
        client->stop();
    }
    thread->join();
}

int TransportWebSocket::send(const std::string& buf, Flags flags) {
    websocketpp::lib::error_code ec;
    // printf("%s:%i size: %li %s\n", __PRETTY_FUNCTION__, __LINE__, buf.size(), websocketpp::utility::to_hex(buf).c_str());
    if (server) {
        auto lock = connections.lockedAccess();
        for (auto &listener : lock.get()) {
            server->send(listener, buf, opcode, ec);
        }
    } else if (clientConnected) {
        client->send(clientConnection, buf, opcode, ec);
    }
    if (ec) {
        std::cout << "> send error: " << ec.message() << std::endl;
        throw std::exception();
    }
    return buf.size();
}

int TransportWebSocket::receive(std::string* buf, Flags flags) {
    int res = 0;
    auto queue = recvQueue.lockedAccess();
    if (queue->size()) {
        *buf = queue->front();
        queue->pop();
        return buf->size();
    }
    return 0;
}
