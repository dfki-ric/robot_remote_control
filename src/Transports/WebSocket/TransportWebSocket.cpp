#include "TransportWebSocket.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <experimental/filesystem>

using namespace std;
using namespace robot_remote_control;

namespace fs = std::experimental::filesystem;

namespace {

std::string urlDecode(const std::string &encoded) {
    std::string decoded;
    for (size_t i = 0; i < encoded.size(); ++i) {
        if (encoded[i] == '%' && i + 2 < encoded.size() && isxdigit(encoded[i + 1]) && isxdigit(encoded[i + 2])) {
            decoded += static_cast<char>(std::stoi(encoded.substr(i + 1, 2), nullptr, 16));
            i += 2;
        } else {
            decoded += encoded[i];
        }
    }
    return decoded;
}

std::string mimeType(const std::string &path) {
    static const std::map<std::string, std::string> types = {
        {"html", "text/html"}, {"htm", "text/html"}, {"js", "text/javascript"}, {"css", "text/css"},
        {"json", "application/json"}, {"txt", "text/plain"}, {"xml", "application/xml"}, {"urdf", "application/xml"},
        {"svg", "image/svg+xml"}, {"png", "image/png"}, {"jpg", "image/jpeg"}, {"jpeg", "image/jpeg"}, {"ico", "image/x-icon"}};
    // just assume the file ending is correct
    std::string ending = fs::path(path).extension().string();
    if (ending.size()) {
        ending = ending.substr(1);
    }
    std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
    auto type = types.find(ending);
    if (type != types.end()) {
        return type->second;
    }
    return "application/octet-stream";
}

std::string htmlPage(const std::string &title, const std::string &body) {
    return "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\"><title>" + title + "</title></head><body>" + body + "</body></html>";
}

}  // namespace

//https://github.com/zaphoyd/websocketpp/blob/master/examples/telemetry_server/telemetry_server.cpp

TransportWebSocket::TransportWebSocket(const ConnectionType &type, const int &port, const std::string &addr):clientConnected(false) {

    if (type == SERVER_TEXT || type == CLIENT_TEXT) {
        opcode = websocketpp::frame::opcode::text;
        serialization.setMode(Serialization::JSON);
    } else {
        opcode = websocketpp::frame::opcode::binary;
        serialization.setMode(Serialization::BINARY);
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

        // plain http requests (no websocket upgrade), for folders added by serveFolder()
        server->set_http_handler([this](websocketpp::connection_hdl hdl) {
            handleHttpRequest(hdl);
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

void TransportWebSocket::setSerializationMode(const Serialization::Mode & mode) {
    serialization.setMode(mode);
    if (mode == Serialization::JSON) {
        opcode = websocketpp::frame::opcode::text;
        serialization.setMode(Serialization::JSON);
    } else {
        opcode = websocketpp::frame::opcode::binary;
        serialization.setMode(Serialization::BINARY);
    }
}

void TransportWebSocket::serveFolder(const std::string& folderpath, const std::string& url) {
    if (!server) {
        printf("TransportWebSocket: serveFolder() is only supported by servers\n");
        return;
    }
    // the url is a single path segment
    std::string key = url;
    key.erase(std::remove(key.begin(), key.end(), '/'), key.end());
    servedFolders.lockedAccess().get()[key] = folderpath;
}

void TransportWebSocket::handleHttpRequest(websocketpp::connection_hdl hdl) {
    WSServer::connection_ptr connection = server->get_con_from_hdl(hdl);
    auto reply = [&connection](const websocketpp::http::status_code::value &status, const std::string &body, const std::string &mime) {
        connection->set_status(status);
        connection->set_body(body);
        connection->replace_header("Content-Type", mime);
    };

    if (connection->get_request().get_method() != "GET") {
        reply(websocketpp::http::status_code::method_not_allowed, "405", "text/plain");
        return;
    }

    // split the path, without query and fragment
    std::string resource = connection->get_resource();
    resource = urlDecode(resource.substr(0, resource.find_first_of("?#")));
    std::vector<std::string> path;
    std::stringstream segments(resource);
    std::string segment;
    while (std::getline(segments, segment, '/')) {
        if (segment == "..") {
            // no access outside of the served folders
            reply(websocketpp::http::status_code::forbidden, "403", "text/plain");
            return;
        }
        if (!segment.empty() && segment != ".") {
            path.push_back(segment);
        }
    }

    const std::map<std::string, std::string> folders = servedFolders.lockedAccess().get();

    if (path.empty()) {
        std::string links;
        for (const auto &folder : folders) {
            links += "<a href=\"/" + folder.first + "/\">" + folder.first + "</a><br>";
        }
        reply(websocketpp::http::status_code::ok, htmlPage("robot_remote_control", links), "text/html");
        return;
    }

    auto folder = folders.find(path[0]);
    if (folder == folders.end()) {
        reply(websocketpp::http::status_code::not_found, "404", "text/plain");
        return;
    }
    std::string localpath = folder->second;
    std::string uripath = "/" + path[0] + "/";
    for (size_t i = 1; i < path.size(); ++i) {
        localpath += "/" + path[i];
        uripath += path[i] + "/";
    }

    std::error_code error;
    if (fs::is_directory(localpath, error)) {
        std::vector<std::string> entries;
        for (const auto &entry : fs::directory_iterator(localpath, error)) {
            std::string name = entry.path().filename().string();
            if (fs::is_directory(entry.path(), error)) {
                name += "/";
            }
            entries.push_back(name);
        }
        std::sort(entries.begin(), entries.end());
        std::string links;
        for (const std::string &name : entries) {
            links += "<a href=\"" + uripath + name + "\">" + name + "</a><br>";
        }
        reply(websocketpp::http::status_code::ok, htmlPage("Directory listing", links), "text/html");
        return;
    }

    std::ifstream file(localpath, std::ios::in | std::ios::binary);
    if (!fs::is_regular_file(localpath, error) || !file) {
        reply(websocketpp::http::status_code::not_found, "404", "text/plain");
        return;
    }
    std::stringstream content;
    content << file.rdbuf();
    reply(websocketpp::http::status_code::ok, content.str(), mimeType(localpath));
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
    return receiveMessage(buf, flags) ? buf->size() : 0;
}

bool TransportWebSocket::receiveMessage(std::string* buf, Flags flags) {
    auto queue = recvQueue.lockedAccess();
    if (queue->size()) {
        *buf = queue->front();
        queue->pop();
        return true;
    }
    return false;
}
