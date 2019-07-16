#pragma once 

#include <string>
#include <memory>

namespace controlledRobot
{

    

    class Transport{
        public: 
        
        enum Flags {NONE = 0x0, NOBLOCK = 0x1};

        Transport(){};
        virtual ~Transport(){};

        /**
         * @brief send date
         * 
         * @param buf the buffer to send
         * @param Flags flags the flags
         * @return int number of bytes sent
         */
        virtual int send(const std::string& buf, Flags flags = NONE) = 0;

        /**
         * @brief receive data
         * 
         * @param buf buffer to fill on receive
         * @param Flags flags the flags
         * @return int 0 if no data received, size of data otherwise
         */
        virtual int receive(std::string* buf, Flags flags = NONE) = 0;

    };

    typedef std::shared_ptr<Transport> TransportSharedPtr;

}