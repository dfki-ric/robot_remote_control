#pragma once 

#include <string>

namespace interaction
{

    class Transport{
        public: 

        Transport(){};
        virtual ~Transport(){};
        /**
         * @brief send a request on a "stable"
         * 
         * @param buf 
         * @return int 
         */
        virtual int send(const std::string& buf) = 0;

        virtual std::string receive() = 0;






    };

}