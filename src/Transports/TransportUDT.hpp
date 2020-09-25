#pragma once

#include <thread>

#include "robot_remote_control/Transports/Transport.hpp"
#include "robot_remote_control/UpdateThread/ThreadProtectedVar.hpp"

#include <arpa/inet.h>
#include "udt/udt.h"

namespace robot_remote_control
{
    class TransportUDT : public Transport
    {
        public: 

            enum ConnectionType {SERVER,CLIENT};

            TransportUDT(const ConnectionType &type, const int &port, const std::string &addr = "", size_t recvBufferSize=10000000);
            virtual ~TransportUDT();

            /**
             * @brief send date
             * 
             * @param buf the buffer to send
             * @param Flags flags the flags
             * @return int number of bytes sent
             */
            virtual int send(const std::string& buf, Flags flags = NONE);

            /**
             * @brief receive data
             * 
             * @param buf buffer to fill on receive
             * @param Flags flags the flags
             * @return int 0 if no data received, size of data otherwise
             */
            virtual int receive(std::string* buf, Flags flags = NONE);


            private:
                std::thread acceptthread;
                void accept();

                void connect();

                UDTSOCKET serv;

                ThreadProtectedVar<UDTSOCKET> socket;

                ConnectionType connectiontype;
                std::string addr;
                int port;

                bool blockingSend;
                bool blockingRecv;

                size_t recvBufferSize;
                std::string recvBuffer;

    };

} // end namespace robot_remote_control-transport_udt


