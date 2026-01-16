#include "TransportHttp.hpp"

#include <future>

using namespace robot_remote_control;

TransportHttp::TransportHttp(const std::string& url, const ConnectionType& mode):Transport() {
    serialization.setMode(Serialization::JSON);

    if (mode == CLIENT) {
        setTransportSupport(CONTOLLERCOMMANDS);

        web::http::client::http_client_config clientconf;
        clientconf.set_validate_certificates(false);
        clientconf.set_timeout(std::chrono::seconds(10)); 
        // web::credentials clientcred(_XPLATSTR("user"), _XPLATSTR("pass"));
        // clientconf.set_credentials(clientcred);
        client = std::unique_ptr<web::http::client::http_client>(new web::http::client::http_client(_XPLATSTR(url), clientconf));
    } else {
        setTransportSupport(ROBOTCOMMANDS);

        server = std::unique_ptr<rest_api::RestServer>(new rest_api::RestServer(url));
        server->registerPostCallback("command", [&](web::http::http_request& message) {
            //set recvQueue
            std::shared_ptr<std::promise<std::string>> promise = std::make_shared<std::promise<std::string>>();
            std::future<std::string> reply = promise->get_future();

            Request r;
            r.request = message.extract_string().get();
            r.reply = promise;

            recvQueue.lockedAccess()->push(r);

            //wait for Robotcontroller process to process the request
            reply.wait();

            return rest_api::Response(status_codes::OK, reply.get());
        });

        // these get funcuins are for browser access only

        server->registerGetCallback("controllableJoints", [&](rest_api::RestServer::GetQuery query, web::http::http_request& message) {
            // controllableJoints doesn't habe channels
            // if (query["channel"] > 0) {
            // }
            std::future<std::string> reply = requestTelemetry(CONTROLLABLE_JOINTS);
            //wait for Robotcontroller process to process the request
            reply.wait();

            return rest_api::Response(status_codes::OK, reply.get());
        });


        server->startListen();
    }
    

}

TransportHttp::~TransportHttp() {

}

std::future<std::string> TransportHttp::requestTelemetry(const TelemetryMessageType type, const int &channel) {
    std::shared_ptr<std::promise<std::string>> promise = std::make_shared<std::promise<std::string>>();

    Request r;

    ControlMessage control;
    control.set_type(TELEMETRY_REQUEST);
    TelemetryRequest telemetryreq;
    telemetryreq.set_type(type);
    telemetryreq.set_channel(channel);

    serialization.serialize(telemetryreq, &control);
    serialization.serialize(control, &r.request);
    r.reply = promise;

    recvQueue.lockedAccess()->push(r);

    return promise->get_future();
}


int TransportHttp::send(const std::string& buf, Flags flags) {
    if (server) {
        // in controller send comed after receive
        activeRequest.reply->set_value(buf);
    }
    if (client) {
        web::http::http_response response = client->request(web::http::methods::POST, "/api/command", buf, "text/json").get();
        if (response.status_code() == status_codes::OK) {
            // we now already received the answer, store it for the next get call:
            clientRecvQueue.lockedAccess()->push(response.extract_string().get());
        }else{
            printf("connection lost");
        }
    }
    return buf.size();
}

int TransportHttp::receive(std::string* buf, Flags flags) {
    if (server) {
        // forward POST requests 
        int res = 0;
        auto queue = recvQueue.lockedAccess();
        if (queue->size()) {
            // todo use conditionvar to block?
            activeRequest = queue->front();
            *buf = activeRequest.request;
            queue->pop();
            return buf->size();
        }
    }
    if (client) {
        auto queue = clientRecvQueue.lockedAccess();
        if (queue->size()) {
            *buf = queue->front();
            queue->pop();
            return buf->size();
        }
    }

    return 0;
}







