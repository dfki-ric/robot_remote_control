#pragma once 

#include "Transport.hpp"
#include <memory>
#include <string>


namespace zmq{
    class context_t;
    class socket_t;
}


namespace interaction
{

    class TransportZmq: public Transport{

        public:

        enum ConnectionType {REQ,REP,PUB,SUB};

        TransportZmq(const std::string &addr, const ConnectionType &type);
        virtual ~TransportZmq(){};

        int send(const std::string& buf);

        std::string receive();


        private:

            std::shared_ptr<zmq::context_t> context;
	        std::shared_ptr<zmq::socket_t> socket;

    };
}