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

    bool hasChannelBuffer(const uint16_t &type, const uint8_t &channel) {
        auto lockedAccessObject = lockedAccess();
        return existingBuffers[type][channel];
    }

    /**
     * @brief 
     * 
     * @param type 
     * @param channel 
     * @param buffersize 
     * @return true 
     * @return false 
     */
    bool addChannelBuffer(const uint16_t &type, const uint8_t &channel, const size_t &buffersize) {
        auto lockedAccessObject = lockedAccess();
        return addChannelBufferInternal(type, channel, buffersize, lockedAccessObject.get()[type]);
    }


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

    class BufferFactroyBase {
     public:
        virtual ~BufferFactroyBase() {}
        virtual std::shared_ptr<RingBufferBase> create(const size_t &buffersize = 10) = 0;
    };
    template <class PBTYPE> class BufferFactroy : public BufferFactroyBase {
     public:
        BufferFactroy(){}
        virtual ~BufferFactroy() {}
        virtual std::shared_ptr<RingBufferBase> create(const size_t &buffersize = 10) {
            return std::shared_ptr<RingBufferBase>(new RingBuffer<PBTYPE>(buffersize));
        }
    };


    template<class PBTYPE> uint8_t registerType(const uint16_t &type, const size_t &buffersize) {
        auto lockedAccessObject = lockedAccess();

        // create a bufferfactory
        if (bufferFactroies.size() <= type) {
            bufferFactroies.resize(type + 1);  // size != index
        }
        std::shared_ptr<BufferFactroyBase> bufferfactory = std::make_shared< BufferFactroy<PBTYPE> >();
        bufferFactroies[type] = bufferfactory;

        // add actual buffer type
        if (lockedAccessObject.get().size() <= type) {  // if size == type, index of type is not available
            lockedAccessObject.get().resize(type + 1);  // size != index
        }
        // init the default channel (0) buffer
        int nextchannelno = lockedAccessObject.get()[type].size(); // the controlled_robot may call registerType in add_channel
        addChannelBufferInternal(type, nextchannelno, buffersize, lockedAccessObject.get()[type]);
        // lockedAccessObject.get()[type].push_back(bufferfactory->create());
        

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

        uint8_t channelno = lockedAccessObject.get()[type].size()-1;
        return channelno;
    }

 private:
    /**
     * @brief non-locked version
     * 
     * @param type 
     * @param channel 
     * @return true 
     * @return false 
     */
    bool addChannelBufferInternal(const uint16_t &type, const uint8_t &channel, const size_t &buffersize, std::vector <std::shared_ptr <RingBufferBase> > &channels) {
        if (existingBuffers[type][channel]) {
            return false;
        }
        if (channels.size() <= channel) {
            channels.resize(channel + 1); // size != channelno (channel 0 == size 1)
        }
        channels[channel] = bufferFactroies[type]->create(buffersize);
        if (existingBuffers.size() <= type) {
            existingBuffers.resize(type +1);
        }
        if (existingBuffers[type].size() <= channel){
            existingBuffers[type].resize(channel + 1);
        }
        existingBuffers[type][channel] = true;
        return true;
    }

    std::vector< std::shared_ptr<ProtobufToStringBase> > converters;
    std::vector< std::shared_ptr<StringToProtobufBase> > convertersToProto;
    std::vector< std::shared_ptr<BufferFactroyBase> > bufferFactroies;
    std::vector< std::vector<bool> > existingBuffers;
};

}  // namespace robot_remote_control
