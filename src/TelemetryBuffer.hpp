#pragma once


#include <memory>
#include <vector>
#include <string>

#include "UpdateThread/LockableClass.hpp"
#include "RingBuffer.hpp"
#include "MessageTypes.hpp"


namespace robot_remote_control {

/**
 * @brief Buffer implementation for all types and channels
 * e.g.: telemetryBuffer[MESSGAETYLE][CHANNEL]->push()
 * 
 */
class TelemetryBuffer: public LockableClass< std::vector < std::vector <std::shared_ptr <RingBufferBase> > > >{
 public:
    TelemetryBuffer();

    ~TelemetryBuffer();

    /**
     * @brief get the serializes buffer value, so the calling function does not need to know the datatype.
     * 
     * @param type The TelemetryMessageType, using a size_t to be able to extent the function
     * @return std::string 
     */
    std::string peekSerialized(const uint16_t &type, const uint8_t &channel = 0);

    bool pushSerialized(const uint16_t &type, const std::string& data, const uint8_t &channel = 0);


    class ProtobufToStringBase {
     public:
        virtual ~ProtobufToStringBase() {}
        virtual std::string get(const uint8_t &channel = 0) = 0;
    };

    template <class PBTYPE> class ProtobufToString : public ProtobufToStringBase {
     public:
        explicit ProtobufToString(const uint16_t& type, TelemetryBuffer *telemetrybuffer) : type(type), telemetrybuffer(telemetrybuffer) {}
        virtual ~ProtobufToString() {}

        virtual std::string get(const uint8_t &channel = 0) {
            std::string buf("");
            PBTYPE data;
            // expects the buffer to be locked!
            auto lockObj = telemetrybuffer->lockedAccess();
            if (RingBufferAccess::peekData(lockObj.get()[type][channel], &data)) {
                data.SerializeToString(&buf);
            }
            return buf;
        }

     private:
        uint16_t type;
        TelemetryBuffer* telemetrybuffer;
    };

    class StringToProtobufBase {
     public:
        virtual ~StringToProtobufBase() {}
        virtual bool set(const std::string& buf, const uint8_t &channel = 0) = 0;
    };

    template <class PBTYPE> class StringToProtobuf : public StringToProtobufBase {
     public:
        explicit StringToProtobuf(const uint16_t& type, TelemetryBuffer *telemetrybuffer) : type(type), telemetrybuffer(telemetrybuffer) {}
        virtual ~StringToProtobuf() {}

        virtual bool set(const std::string& buf, const uint8_t &channel = 0) {
            PBTYPE data;
            data.ParseFromString(buf);
            // expects the buffer to be locked!
            auto lockObj = telemetrybuffer->lockedAccess();
            return RingBufferAccess::pushData(lockObj.get()[type][channel], data);
        }

     private:
        uint16_t type;
        TelemetryBuffer* telemetrybuffer;
    };

    template<class PBTYPE> void registerType(const uint16_t &type, const size_t &buffersize) {
        auto lockedAccessObject = lockedAccess();

        // add buffer type
        if (lockedAccessObject.get().size() <= type) {  // if size == type, index of type is not available
            lockedAccessObject.get().resize(type + 1);  // size != index
        }

        std::shared_ptr<RingBufferBase> newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<PBTYPE>(buffersize));
        lockedAccessObject.get()[type].push_back(newbuf);

        // add toString converter
        if (converters.size() <= type) {
            converters.resize(type + 1);  // size != index
        }
        std::shared_ptr<ProtobufToStringBase> conf = std::make_shared< ProtobufToString<PBTYPE> >(type, this);
        converters[type] = conf;


        // add toProtobuf converter
        if (convertersToProto.size() <= type) {
            convertersToProto.resize(type + 1);  // size != index
        }
        std::shared_ptr<StringToProtobufBase> conv = std::make_shared< StringToProtobuf<PBTYPE> >(type, this);
        convertersToProto[type] = conv;
    }



 private:
    std::vector< std::shared_ptr<ProtobufToStringBase> > converters;
    std::vector< std::shared_ptr<StringToProtobufBase> > convertersToProto;
};

}  // namespace robot_remote_control
