#pragma once


#include <memory>
#include <vector>
#include <string>

#include "UpdateThread/ThreadProtectedVar.hpp"
#include "RingBuffer.hpp"
#include "MessageTypes.hpp"


namespace robot_remote_control {

class TelemetryBuffer: public ThreadProtectedVar< std::vector < std::shared_ptr <RingBufferBase> > > {
    public:
        explicit TelemetryBuffer(const size_t &size = 1);

        ~TelemetryBuffer();


        /**
         * @brief get the serializes buffer value, so the calling function does not need to know the datatype.
         * 
         * @param type 
         * @return std::string 
         */
        std::string peekSerialized(const TelemetryMessageType &type);

    private:
        template <class PROTO> void fillBuffer(const TelemetryMessageType &type, PROTO *proto, std::string *targetbuffer){
            if (RingBufferAccess::peekData(get_ref()[type], proto)){
                proto->SerializeToString(targetbuffer);
            }
        }
};

}  // namespace robot_remote_control
