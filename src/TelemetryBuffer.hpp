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
         * @param type The TelemetryMessageType, using a size_t to be able to extent the function
         * @return std::string 
         */
        virtual std::string peekSerialized(const uint16_t &type);

    protected:
        template <class PROTO> void fillBuffer(const uint16_t &type, PROTO *proto, std::string *targetbuffer){
            if (RingBufferAccess::peekData(get_ref()[type], proto)){
                proto->SerializeToString(targetbuffer);
            }
        }
};

}  // namespace robot_remote_control
