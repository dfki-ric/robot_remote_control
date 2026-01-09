#pragma once

#include "Types/RobotRemoteControl.pb.h"
#include <google/protobuf/util/json_util.h>

namespace robot_remote_control {

class Serialization {
 public:

    enum Mode{BINARY, JSON};

    Serialization(){
        // jsonOptions.add_whitespace = true;
        jsonOptions.always_print_primitive_fields = true;
        serializationMode = JSON;
    }
    virtual ~Serialization(){}


    void setMode(const Mode & mode) {
        serializationMode = mode;
    }
    Mode getMode() {
        return serializationMode;
    }

    template<class PROTO> bool serialize(const PROTO& src, std::string *target) {
        bool res = false;
        if (serializationMode == JSON) {
            if (google::protobuf::util::MessageToJsonString(src, target, jsonOptions).ok()){
                res = true;
            }
        }else{
            res = src.SerializeToString(target);
        }
        return res;
    }

    template<class PROTO> bool serialize(const PROTO& src, TelemetryMessage *target) {
        bool res = false;
        if (serializationMode == JSON) {
            if (google::protobuf::util::MessageToJsonString(src, target->mutable_json(), jsonOptions).ok()){
                res = true;
            }
        }else{
            res = src.SerializeToString(target->mutable_data());
        }
        return res;
    }

    template<class PROTO> bool serialize(const PROTO& src, ControlMessage *target) {
        bool res = false;
        if (serializationMode == JSON) {
            if (google::protobuf::util::MessageToJsonString(src, target->mutable_json(), jsonOptions).ok()){
                res = true;
            }
        }else{
            res = src.SerializeToString(target->mutable_data());
        }
        return res;
    }

    template<class PROTO> void setSerialized(const std::string &serialized, PROTO *target) {
        if (serializationMode == JSON) {
            target->set_json(serialized);
        }else{
            target->set_data(serialized);
        }
    }

    template<class PROTO> size_t getPayloadSize(const PROTO& message) {
        if (serializationMode == JSON) {
            return message.json().size();
        }else{
            return message.data().size();
        }
        return 0;
    }

    bool deserialize(const std::string& src, TelemetryMessage* target, std::string *serializedMessage) {
        bool res = false;
        if (serializationMode == JSON) {
            if(google::protobuf::util::JsonStringToMessage(src, target).ok()) {
                *serializedMessage = target->json();
                res = true;
            }
        } else {
            res = target->ParseFromString(src);
            *serializedMessage = target->data();
        }
        return res;
    }

    template<class PROTO> bool deserialize(const std::string& src, PROTO* target) {
        bool res = false;
        if (serializationMode == JSON) {
            if(google::protobuf::util::JsonStringToMessage(src, target).ok()) {
                res = true;
            }
        } else {
            res = target->ParseFromString(src);
        }
        return res;
    }


    template<class PROTO> bool deserializeLongData(const std::string& src, PROTO* target) {
        bool res = false;
        if (serializationMode == JSON) {
            if(google::protobuf::util::JsonStringToMessage(src, target).ok()) {
                res = true;
            }
        } else {
            google::protobuf::io::CodedInputStream cistream(reinterpret_cast<const uint8_t *>(src.data()), src.size());
            cistream.SetTotalBytesLimit(src.size());
            res = target->ParseFromCodedStream(&cistream);
        }
        return res;
    }

 private:
    Mode serializationMode;
    google::protobuf::util::JsonPrintOptions jsonOptions;
};

}  // namespace robot_remote_control
