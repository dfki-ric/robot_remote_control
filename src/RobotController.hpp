#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"

namespace interaction
{
    class RobotController
    {
        public: 

            RobotController(std::shared_ptr<interaction::Transport> commandTransport,std::shared_ptr<interaction::Transport> telemetryTransport);
            virtual ~RobotController(){};
            
            int setTargetPose(const interaction::Pose & pose);

            int setTwistCommand(const interaction::Twist &twistCommand);

            

            
            interaction::Pose getCurrentPose();

        protected:
            virtual std::string sendRequest(const std::string& serializedMessage);

            ControlMessageType evaluateReply(const std::string& reply);

        private:

            std::shared_ptr<interaction::Transport> commandTransport;
            std::shared_ptr<interaction::Transport> telemetryTransport;

            interaction::Pose currentPose;


            template< class CLASS > std::string sendProtobufData(CLASS protodata, const ControlMessageType &type ){
                std::string buf;
                buf.resize(sizeof(uint16_t));
                uint16_t uint_type = type;
                uint16_t* data = (uint16_t*)buf.data();
                *data = uint_type;
                protodata.AppendToString(&buf);
                return sendRequest(buf);
            }


    };

} // end namespace interaction-library-controlled_robot

