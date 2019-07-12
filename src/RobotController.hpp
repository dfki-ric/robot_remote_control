#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"
#include "RingBuffer.hpp"
#include "UpdateThread/UpdateThread.hpp"

namespace interaction
{
    class RobotController: public UpdateThread
    {
        public: 

            RobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport = TransportSharedPtr());
            virtual ~RobotController();
            
            /**
             * @brief in case there is a telemetry connection, receive all and fill the data fields
             */
            virtual void update();


            void setTargetPose(const interaction::Pose & pose);

            void setTwistCommand(const interaction::Twist &twistCommand);

            unsigned int getBufferSize(const TelemetryMessageType &type){
                buffers.lock();
                int size = buffers.get_ref()[type]->size();
                buffers.unlock();
                return size;
            }


            template< class DATATYPE > unsigned int getTelemetry(const TelemetryMessageType &type, DATATYPE &data ){
                buffers.lock();
                bool result = RingBufferAccess::popData(buffers.get_ref()[type],data);
                buffers.unlock();
                return result;
            }
            
            unsigned int getCurrentPose(interaction::Pose & pose){
                return getTelemetry(CURRENT_POSE,pose);
            }

            unsigned int getCurrentJointState(interaction::JointState & jointState){
                return getTelemetry(JOINT_STATE,jointState);
            }



        protected:
            virtual std::string sendRequest(const std::string& serializedMessage);

            ControlMessageType evaluateReply(const std::string& reply);

        private:

            TransportSharedPtr commandTransport;
            TransportSharedPtr telemetryTransport;

            

            ThreadProtecetedVar< std::vector< std::shared_ptr<RingBufferBase> > > buffers;
            void initBuffers(const unsigned int &defaultSize);


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

