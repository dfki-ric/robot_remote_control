#pragma once

#include "UpdateThread/ThreadProtectedVar.hpp"
#include "RingBuffer.hpp"
#include "MessageTypes.hpp"

namespace controlledRobot
{

class TelemetryBuffer: public ThreadProtecetedVar< std::vector< std::shared_ptr<RingBufferBase> > >{
    public:
    
    TelemetryBuffer(const size_t &size = 1);

    ~TelemetryBuffer();


    std::string peekSerialized(const TelemetryMessageType &type);
    

    private:

    template <class PROTO> void fillBuffer(const TelemetryMessageType &type, PROTO &proto, std::string &targetbuffer){
        if (RingBufferAccess::peekData(get_ref()[type],proto)){
            proto.SerializeToString(&targetbuffer);
        };
    };

};

}