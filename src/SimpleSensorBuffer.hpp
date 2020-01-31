#pragma once

#include "UpdateThread/ThreadProtectedVar.hpp"
#include "RingBuffer.hpp"
#include "MessageTypes.hpp"

#include <memory>
#include <vector>

namespace robot_remote_control {

class SimpleSensorBuffer: public ThreadProtectedVar< std::vector< std::shared_ptr<RingBufferBase> > >{
    public:
        SimpleSensorBuffer();

        ~SimpleSensorBuffer() {}



        void initBufferID (const uint16_t &id);

        // size_t size(){
        //     return get().size();
        // }


        private:
            // void resize(const size_t &size);
        //std::string peekSerialized(const TelemetryMessageType &type);
    

        // private:

        // template <class PROTO> void fillBuffer(const TelemetryMessageType &type, PROTO &proto, std::string &targetbuffer){
        //     if (RingBufferAccess::peekData(get_ref()[type],proto)){
        //         proto.SerializeToString(&targetbuffer);
        //     };
        // };

};

}  // namespace robot_remote_control