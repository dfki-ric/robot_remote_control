#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"
#include "TelemetryBuffer.hpp"
#include "UpdateThread/UpdateThread.hpp"


namespace controlledRobot
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

            /**
             * @brief Set the Target Pose of the ControlledRobot
             * 
             * @param pose The Pose the robot should move to
             */
            void setTargetPose(const Pose & pose);

            /**
             * @brief Set the Twist Command for direct remote control based on velocities
             * 
             * @param twistCommand 
             */
            void setTwistCommand(const Twist &twistCommand);

            /**
             * @brief Set the GoTo Command of the ControlledRobot
             *
             * @param goToCommand
             */
            void setGoToCommand(const GoTo &goToCommand);


            /**
             * @brief Get the Number of pending messages for a specific Telemetry type
             * 
             * @param TelemetryMessageType  
             * @return unsigned int number of messages in the buffer
             */
            unsigned int getBufferSize(const TelemetryMessageType &type){
                buffers.lock();
                int size = buffers.get_ref()[type]->size();
                buffers.unlock();
                return size;
            }

            /**
             * @brief Get the TelemetryMessages
             * @warning This should not be called directly
             * 
             * @tparam DATATYPE 
             * @param type 
             * @param data 
             * @return unsigned int 
             */

            template< class DATATYPE > unsigned int getTelemetry(const TelemetryMessageType &type, DATATYPE &data ){
                buffers.lock();
                bool result = RingBufferAccess::popData(buffers.get_ref()[type],data);
                buffers.unlock();
                return result;
            }
            

            /**
             * @brief Get the last sent Pose of the robot
             * 
             * @param pose the pose to write the data to
             * @return unsigned int the number of pending messages in the buffer after read
             */
            unsigned int getCurrentPose(Pose & pose){
                return getTelemetry(CURRENT_POSE,pose);
            }

            /**
             * @brief Get the last sent joint state of the robot
             * 
             * @param pose the pose to write the data to
             * @return unsigned int the number of pending messages in the buffer after read
             */
            unsigned int getCurrentJointState(JointState & jointState){
                return getTelemetry(JOINT_STATE,jointState);
            }



        protected:
            virtual std::string sendRequest(const std::string& serializedMessage);

            ControlMessageType evaluateReply(const std::string& reply);

        private:

            TransportSharedPtr commandTransport;
            TransportSharedPtr telemetryTransport;

            

            TelemetryBuffer buffers;
            //void initBuffers(const unsigned int &defaultSize);


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

