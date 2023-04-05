#pragma once


#include <memory>
#include <vector>
#include <string>

#include "UpdateThread/LockableClass.hpp"
#include "RingBuffer.hpp"
#include "MessageTypes.hpp"


namespace robot_remote_control {

class TelemetryBuffer: public LockableClass< std::vector < std::shared_ptr <RingBufferBase> > > {
 public:
    explicit TelemetryBuffer();

    ~TelemetryBuffer();


    /**
     * @brief get the serializes buffer value, so the calling function does not need to know the datatype.
     * 
     * @param type The TelemetryMessageType, using a size_t to be able to extent the function
     * @return std::string 
     */
    std::string peekSerialized(const uint16_t &type);

    bool pushSerialized(const uint16_t &type, const std::string& data);


    class ProtobufToStringBase {
     public:
        virtual ~ProtobufToStringBase() {}
        virtual std::string get() = 0;
    };

    template <class PBTYPE> class ProtobufToString : public ProtobufToStringBase {
     public:
        explicit ProtobufToString(const uint16_t& type, TelemetryBuffer *telemetrybuffer) : type(type), telemetrybuffer(telemetrybuffer) {}
        virtual ~ProtobufToString() {}

        virtual std::string get() {
            std::string buf("");
            PBTYPE data;
            // expects the buffer to be locked!
            auto lockObj = telemetrybuffer->lockedAccess();
            if (RingBufferAccess::peekData(lockObj.get()[type], &data)) {
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
        virtual bool set(const std::string& buf) = 0;
    };

    template <class PBTYPE> class StringToProtobuf : public StringToProtobufBase {
     public:
        explicit StringToProtobuf(const uint16_t& type, TelemetryBuffer *telemetrybuffer) : type(type), telemetrybuffer(telemetrybuffer) {}
        virtual ~StringToProtobuf() {}

        virtual bool set(const std::string& buf) {
            PBTYPE data;
            data.ParseFromString(buf);
            // expects the buffer to be locked!
            auto lockObj = telemetrybuffer->lockedAccess();
            return RingBufferAccess::pushData(lockObj.get()[type], data);
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
        lockedAccessObject.get()[type] = newbuf;

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
