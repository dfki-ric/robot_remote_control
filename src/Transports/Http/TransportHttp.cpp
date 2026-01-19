#include "TransportHttp.hpp"

#include <future>

#include "../TypeGenerator.hpp"

using namespace robot_remote_control;

#define STRING(A) #A

#define REGISTER_REQUEST(URL, DEFINE, DOC) \
    server->registerGetCallback(#URL, [&](rest_api::RestServer::GetQuery query, web::http::http_request& message) { \
        std::future<std::string> reply = requestTelemetry(DEFINE); \
        reply.wait(); \
        return rest_api::Response(status_codes::OK, reply.get(), "application/json"); \
    }, DOC); \



TransportHttp::TransportHttp(const std::string& url, const ConnectionType& mode):Transport() {

    serialization.setMode(Serialization::JSON);
    sampleSerialization.setMode(Serialization::JSON);
    sampleSerialization.setPrintPrimitiveFields(true);
    sampleSerialization.setAddWhitespace(true);

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

            return rest_api::Response(status_codes::OK, reply.get(), "application/json");
        },"generic POST to send ControlMessages to, this could also be used to obtain all the other calls exposed int his api");

        // these get functions are for browser access only

        server->registerGetCallback("telemetry", [&](rest_api::RestServer::GetQuery query, web::http::http_request& message) {

            int typeId = getTelemetryType(query["type"]);
            if (typeId == 0) {
                return rest_api::Response(status_codes::BadRequest, "type could not be parsed");
            }

            TelemetryMessageType type = static_cast<TelemetryMessageType>(typeId);
            std::future<std::string> reply = requestTelemetry(type, std::atoi(query["channel"].c_str()));
            //wait for Robotcontroller thread to process the request
            reply.wait();

            return rest_api::Response(status_codes::OK, reply.get(), "application/json");
        }, "generic telemetry request call /telemetry?type=1&channel=0, for types.");

        // server->registerGetCallback("sampleTelemetry", [&](rest_api::RestServer::GetQuery query, web::http::http_request& message) {
            
        //     int typeId = getTelemetryType(query["type"]);

        //     // TelemetryMessage msg;
        //     // msg.set_type(typeId);

        //     std::string typeStr;
        //     switch (typeId) {
        //         case CURRENT_POSE: sampleSerialization.serialize(Pose(), &typeStr); break;
        //         default: typeStr = "";break;
        //     }
            
        //     // sampleSerialization.serialize(msg, &typeStr);

        //     if (typeStr == "") {
        //         return rest_api::Response(status_codes::BadRequest, "type " + query["type"] + " unknown");
        //     }
        //     return rest_api::Response(status_codes::OK, typeStr);
        // }, "get a json sample of the Telemetry type /sample?type=1, /sample?type=CURRENT_POSE. Will be available stringrified in a TelemetryMassage.json field");

        server->registerGetCallback("sampleCommand", [&](rest_api::RestServer::GetQuery query, web::http::http_request& message) {
            
            ControlMessageType typeId = getControlType(query["type"]);
            
             ControlMessage msg;
            // msg.set_type(typeId);

            std::string typeStr;
            switch (typeId) {
                case PROTOCOL_VERSION:
                case LIBRARY_VERSION:
                case GIT_VERSION: typeStr = "does not need a data payload, just send the type: [PROTOCOL_VERSION, LIBRARY_VERSION, GIT_VERSION]"; break;
                case HEARTBEAT: sampleSerialization.serialize(HeartBeat(), &typeStr); break;
                case TELEMETRY_REQUEST: sampleSerialization.serialize(TelemetryRequest(), &typeStr); break;
                case FILE_REQUEST: sampleSerialization.serialize(FileRequest(), &typeStr); break;
                case LOG_LEVEL_SELECT: sampleSerialization.serialize(LogLevelRequest(), &typeStr); break;
                case PERMISSION: sampleSerialization.serialize(PermissionRequest(), &typeStr); break;
                case TARGET_POSE_COMMAND: sampleSerialization.serialize(Pose(), &typeStr); break;

                case TWIST_COMMAND: sampleSerialization.serialize(TypeGenerator::genTwist(), &typeStr); break;
                case JOINTS_COMMAND: sampleSerialization.serialize(TypeGenerator::genJointCommand(), &typeStr); break;
                case SIMPLE_ACTIONS_COMMAND: sampleSerialization.serialize(TypeGenerator::genSimpleCmd(), &typeStr); break;
                case COMPLEX_ACTION_COMMAND: sampleSerialization.serialize(TypeGenerator::genComplexActions(), &typeStr); break;
                case GOTO_COMMAND: sampleSerialization.serialize(TypeGenerator::genGoTo(), &typeStr); break;
                case ROBOT_TRAJECTORY_COMMAND: sampleSerialization.serialize(TypeGenerator::genPoses(), &typeStr); break;
                default: typeStr = "";break;
            }

            // sampleSerialization.serialize(msg, &typeStr);

            if (typeStr == "") {
                return rest_api::Response(status_codes::BadRequest, "type " + query["type"] + " unknown");
            }
            return rest_api::Response(status_codes::OK, typeStr, "application/json");
        }, "get a json sample of the Telemetry type /sample?type=1, /sample?type=CURRENT_POSE. Must be placed stringrified (escaped) in a ControlMessage.json field \n \
         {  \"type\": <ControlMessageType>, \"json\": <ControlMessage> }");


        REGISTER_REQUEST(robotName, ROBOT_NAME, "The robot name");
        REGISTER_REQUEST(options, INTERFACE_OPTIONS, "Options set by the robot");

        REGISTER_REQUEST(controllableJoints, CONTROLLABLE_JOINTS, "Joints that can be controlled");
        REGISTER_REQUEST(controllableFrames, CONTROLLABLE_FRAMES, "Frames that can be controlled via Pose/Twist");

        REGISTER_REQUEST(channelsDefinition, CHANNELS_DEFINITION, "Definition of sub-channesl added be the ControlledRobot");
        REGISTER_REQUEST(complexActions, COMPLEX_ACTIONS, "The complex actions usable for the robot");
        REGISTER_REQUEST(simpleActions, SIMPLE_ACTIONS, "simple actions supported by the robot");
        REGISTER_REQUEST(videoStreams, VIDEO_STREAMS, "video stream urls");
        
        REGISTER_REQUEST(robotState, ROBOT_STATE, "the current robot state");
        REGISTER_REQUEST(cameraInformation, CAMERA_INFORMATION, "information about the cameras used");

        REGISTER_REQUEST(fileDefinition, FILE_DEFINITION, "named file definitions (name required as paramater for requestFile)");

        server->startListen();
    }
    

}

TransportHttp::~TransportHttp() {

}

bool TransportHttp::serveFolder(const std::string& folderpath, const std::string& url) {
    server->serveFolder(folderpath, url);
}


TelemetryMessageType TransportHttp::getTelemetryType(const std::string &param) {
    // check if type is a number (o=error, of NO_TELEMETRY)
    TelemetryMessageType type = static_cast<TelemetryMessageType>(std::atoi(param.c_str()));
    if (type == 0) {
        // try parsing by text
        if (!TelemetryMessageType_Parse(param, &type) ){
            //no success, type set not set correctly
            return NO_TELEMETRY_DATA;
        }
    }
    return type;
}

ControlMessageType TransportHttp::getControlType(const std::string &param) {
    // check if type is a number (o=error, of NO_TELEMETRY)
    ControlMessageType type = static_cast<ControlMessageType>(std::atoi(param.c_str()));
    if (type == 0) {
        // try parsing by text
        if (!ControlMessageType_Parse(param, &type) ){
            //no success, type set not set correctly
            return NO_CONTROL_DATA;
        }
    }
    return type;
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







