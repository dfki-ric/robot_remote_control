#include "TransportHttp.hpp"

#include <future>

using namespace robot_remote_control;

TransportHttp::TransportHttp(const std::string& url):Transport() {
    server = std::unique_ptr<rest_api::RestServer>(new rest_api::RestServer(url));
    setTransportSupport(ROBOTCOMMANDS);

    serialization.setMode(Serialization::JSON);

    server->registerPostCallback("command", [&](web::http::http_request& message) {
        printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
        // web::json::value json = message.extract_json().get();
        // x = json["x"].as_integer();
        // y = json["y"].as_integer();
        
        //set recvQueue
        // 
        std::shared_ptr<std::promise<std::string>> promise = std::make_shared<std::promise<std::string>>();
        std::future<std::string> reply = promise->get_future();

        Request r;
        r.request = message.extract_string().get();
        r.reply = promise;
        
        printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__, r.request.c_str());

        recvQueue.lockedAccess()->push(r);

        //wait for Robotcontroller process to process the request
        reply.wait();

        return rest_api::Response(status_codes::OK, reply.get());
    });


    server->registerGetCallback("controllableJoints", [&](rest_api::RestServer::GetQuery query, web::http::http_request& message) {
        printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
        
        // controllableJoints doesn't habe chennels
        // if (query["channel"] > 0) {
        // }

        std::future<std::string> reply = requestTelemetry(CONTROLLABLE_JOINTS);
        //wait for Robotcontroller process to process the request
        reply.wait();

        return rest_api::Response(status_codes::OK, reply.get());
    });


    server->startListen();

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
    // in controller send comed after receive
    activeRequest.reply->set_value(buf);
    return buf.size();
}

int TransportHttp::receive(std::string* buf, Flags flags) {

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
    return 0;
}







