#pragma once 

#include <string>
#include <memory>

namespace interaction
{

    

    class Transport{
        public: 
        
        enum Flags {NONE = 0x0, NOBLOCK = 0x1};

        Transport(){};
        virtual ~Transport(){};
        /**
         * @brief send a request on a "stable"
         * 
         * @param buf 
         * @return int 
         */
        virtual int send(const std::string& buf, Flags flags = NONE) = 0;

        /**
         * @brief 
         * 
         * @param buf 
         * @param flags 
         * @return int 0 if no data received
         */
        virtual int receive(std::string* buf, Flags flags = NONE) = 0;

    };

    typedef std::shared_ptr<Transport> TransportSharedPtr;

}