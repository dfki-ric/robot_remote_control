#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"
#include "TelemetryBuffer.hpp"
#include "UpdateThread/UpdateThread.hpp"


namespace robot_remote_control
{
    class RobotController: public UpdateThread
    {
        public: 

            RobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport = TransportSharedPtr(), size_t recv_buffer_size = 10);
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
             * @brief Set the Joint Command object
             * 
             * @param jointsCommand 
             */
            void setJointCommand(const JointState &jointsCommand);

            /**
             * @brief Set the SimpleActions command that the controlled robot should execute
             *
             * @param simpleActionsCommand
             */
            void setSimpleActionCommand(const SimpleAction &simpleActionCommand);

            /**
             * @brief Set the ComplexActions command that the controlled robot should execute
             *
             * @param complexActionsCommand
             */
            void setComplexActionCommand(const ComplexAction &complexActionCommand);

            /**
             * @brief Set the LogLevel of the controlled robot
             * 
             * @param level the desired log level
             */
            void setLogLevel(const uint32_t &level);

            /**
             * @brief Get the last sent Pose of the robot
             * 
             * @param pose the pose to write the data to
             * @return bool true if new data was read
             */
            bool getCurrentPose(Pose & pose){
                return getTelemetry(CURRENT_POSE,pose);
            }

            /**
             * @brief Get the last sent joint state of the robot
             * 
             * @param pose the JointState to write the data to
             * @return bool true if new data was read
             */
            bool getCurrentJointState(JointState & jointState){
                return getTelemetry(JOINT_STATE,jointState);
            }

            /**
             * @brief Get the Log Message object
             * 
             * @param msg LogMessage to write the data to
             * @return bool true if new data was read
             */
            bool getLogMessage(LogMessage & msg){
                return getTelemetry(LOG_MESSAGE,msg);
            }

            /**
             * @brief Get the Robot State
             * 
             * @param state the string to write the state to
             * @return bool true if new data was read
             */
            bool getRobotState(std::string & state){
                RobotState protostate;
                int statesleft = getTelemetry(ROBOT_STATE,protostate);
                state = protostate.state();
                return statesleft;
            }

            /**
             * @brief request the curretn state instead of waiting for the first telemetry message
             * 
             * @param state the string to write the state to
             */
            void requestRobotState(std::string & state){
                RobotState protostate;
                requestTelemetry(ROBOT_STATE, protostate);
                state = protostate.state();
            }

            /**
             * @brief Request information about the complex actions of the robot.
             * 
             * @param complexActions where to write the data to
             * @return void
             */
            void requestComplexActions(ComplexActions &complexActions) {
                requestTelemetry(COMPLEX_ACTIONS, complexActions);
            }

            /**
             * @brief Request information about the simple actions of the robot.
             * 
             * @param simpleActions where to write the data to
             * @return void
             */
            void requestSimpleActions(SimpleActions &simpleActions) {
                requestTelemetry(SIMPLE_ACTIONS, simpleActions);
            }

           /**
             * @brief Request information about the controllable joints of the robot.
             * 
             * @param jointState where to write the data to
             * @return void
             */
            void requestControllableJoints(JointState &jointState) {
                requestTelemetry(CONTROLLABLE_JOINTS, jointState);
            }

            /**
             * @brief Request information about the name of the robot.
             *
             * @param robotName where to write the data to
             * @return void
             */
            void requestRobotName(RobotName &robotName) {
                requestTelemetry(ROBOT_NAME, robotName);
            }

            /**
             * @brief Get the Number of pending messages for a specific Telemetry type
             * 
             * @param TelemetryMessageType  
             * @return unsigned int number of messages in the buffer
             */
            unsigned int getBufferSize(const TelemetryMessageType &type){
                buffers->lock();
                int size = buffers->get_ref()[type]->size();
                buffers->unlock();
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
                buffers->lock();
                bool result = RingBufferAccess::popData(buffers->get_ref()[type],data);
                buffers->unlock();
                return result;
            }

            template< class DATATYPE > void requestTelemetry(const TelemetryMessageType &type, DATATYPE &result){
                std::string buf;
                buf.resize(sizeof(uint16_t)*2);
                uint16_t uint_type;
                uint16_t* data = (uint16_t*)buf.data();

                uint_type = TELEMETRY_REQUEST;
                *data = uint_type;
                
                data++;

                //add the requested type int
                uint_type = type;
                *data = uint_type;

                std::string replybuf = sendRequest(buf);
                result.ParseFromString(replybuf);


            }


        protected:
            virtual std::string sendRequest(const std::string& serializedMessage);

            TelemetryMessageType evaluateTelemetry(const std::string& reply);

        private:

            TransportSharedPtr commandTransport;
            TransportSharedPtr telemetryTransport;

            

            std::shared_ptr<TelemetryBuffer>  buffers;
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

