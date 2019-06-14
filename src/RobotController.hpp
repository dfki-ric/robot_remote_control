#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"

namespace interaction
{
    class RobotController
    {
        public: 

            RobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport = TransportSharedPtr());
            virtual ~RobotController();
            
            /**
             * @brief in case there is a telemetry connection, receive all and fill the data fields
             */
            void updateTelemetry();


            void setTargetPose(const interaction::Pose & pose);

            void setTwistCommand(const interaction::Twist &twistCommand);

            
            interaction::Pose getCurrentPose(){
                return currentPose;
            }

        protected:
            virtual std::string sendRequest(const std::string& serializedMessage);

            ControlMessageType evaluateReply(const std::string& reply);

        private:

            TransportSharedPtr commandTransport;
            TransportSharedPtr telemetryTransport;

            interaction::Pose currentPose;


            template< class CLASS > std::string sendProtobufData(const CLASS &protodata, const ControlMessageType &type ){
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

