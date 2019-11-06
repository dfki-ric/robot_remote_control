#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"
#include "UpdateThread/UpdateThread.hpp"
#include "TelemetryBuffer.hpp"

namespace robot_remote_control
{
    class ControlledRobot: public UpdateThread
    {
        public: 

            ControlledRobot(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport);
            virtual ~ControlledRobot(){};

            /**
             * @brief threaded update function called by UpdateThread that receives commands
             */
            virtual void update();


            //Command getters

            /**
             * @brief Get the Target Pose the robot should move to
             * 
             * @return true if the command was not read before
             * @param command the last received command
             */
            bool getTargetPoseCommand(Pose& command){
                return poseCommand.read(command);
            }

            /**
             * @brief Get the Twist Command with velocities to robe should move at
             * 
             * @return true if the command was not read before
             * @param command the last received command
             */
            bool getTwistCommand(Twist& command){
                return twistCommand.read(command);
            }

            /**
             * @brief Get the GoTo Command the robot should execute
             *
             * @return true if the command was not read before
             * @param command the last received command
             */
            bool getGoToCommand(GoTo &command) {
                return goToCommand.read(command);
            }

            /**
             * @brief Get the Joints Command the robot should execute
             *
             * @return true if the command was not read before
             * @param command the last received command
             */
            bool getJointsCommand(JointState &command) {
                return jointsCommand.read(command);
            }


            /**
             * @brief Get the SimpleActions Command the robot should execute
             *
             * @return true if the command was not read before
             * @param command the last received command
             */
            bool getSimpleActionsCommand(SimpleActions & command) {
                return simpleActionsCommand.read(command);
            }

            /**
             * @brief Get the ComplexActions Command the robot should execute
             *
             * @return true if the command was not read before
             * @param command the last received command
             */
            bool getComplexActionsCommand(ComplexActions &command) {
                return complexActionsCommand.read(command);
            }

            // Telemetry setters

        private:
            /**
             * @brief generic send of telemetry types
             * 
             * @tparam CLASS 
             * @param protodata 
             * @param type 
             * @return int size sent
             */
            template<class CLASS> int sendTelemetry(const CLASS &protodata, const TelemetryMessageType& type){
                if (telemetryTransport.get()){
                    std::string buf;
                    buf.resize(sizeof(uint16_t));
                    uint16_t uint_type = type;
                    uint16_t* data = (uint16_t*)buf.data();
                    *data = uint_type;
                    protodata.AppendToString(&buf);
                    //store latest data for future requests
                    buffers.lock();
                    RingBufferAccess::pushData(buffers.get_ref()[type],protodata, true);
                    buffers.unlock();
                    return telemetryTransport->send(buf) - sizeof(uint16_t);
                }
                printf("ERROR Transport invalid\n");
                return 0;
            };

        public:

            /**
             * @brief Set the current Pose of the robot
             * 
             * @param pose current pose
             * @return int number of bytes sent
             */
            int setCurrentPose(const Pose& telemetry){
                return sendTelemetry(telemetry,CURRENT_POSE);
            }

            /**
             * @brief Set the curretn JointState of the robot
             * 
             * @param pose current JointState
             * @return int number of bytes sent
             */
            int setJointState(const JointState& telemetry){
                 return sendTelemetry(telemetry,JOINT_STATE);
            }

            /**
             * @brief The robot uses this method to provide information about its controllable joints
             *
             * @param controllableJoints the controllable joints of the robot as a JointState
             * @return int number of bytes sent
             */
            int setControllableJoints(const JointState& telemetry){
                return sendTelemetry(telemetry, CONTROLLABLE_JOINTS);
            }

            /**
             * @brief The robot uses this method to provide information about its set of simple actions
             *
             * @param simpleActions the simple actions of the robot to report to the controler
             * the state field of the SimpleActions class should be filled with the max value
             * @return int number of bytes sent
             */
            int setSimpleActions(const SimpleActions& telemetry){
                return sendTelemetry(telemetry, SIMPLE_ACTIONS);
            }

            /**
             * @brief The robot uses this method to provide information about its set of complex actions
             *
             * @param complexActions the complex actions of the robot as a ComplexActions
             * @return int number of bytes sent
             */
            int setComplexActions(const ComplexActions& telemetry){
                return sendTelemetry(telemetry, COMPLEX_ACTIONS);
            }

            /**
             * @brief The robot uses this method to provide information about its name
             *
             * @param robotName the name of the robot as a RobotName
             * @return int number of bytes sent
             */
            int setRobotName(const RobotName& telemetry){
                return sendTelemetry(telemetry, ROBOT_NAME);
            }

            int setRobotState(const std::string& state);

            int setLogMessage(enum LogLevel lvl, const std::string& message);

            int setLogMessage(const LogMessage& log_message);



        protected:
            virtual ControlMessageType receiveRequest();

            virtual ControlMessageType evaluateRequest(const std::string& request);

        private:

            template<class COMMAND> struct CommandBuffer{
                public: 
                    CommandBuffer():isnew(false){};

                    bool read(COMMAND &target){
                        bool oldval = isnew.get();
                        target = command.get();
                        isnew.set(false);
                        return oldval;
                    }

                    void write(COMMAND &src){
                        command.set(src);
                        isnew.set(true);
                    }

                    void write(const std::string &serializedMessage){
                        command.lock();
                        command.get_ref().ParseFromString(serializedMessage);
                        command.unlock();
                        isnew.set(true);
                    }

                private:
                    ThreadProtecetedVar<COMMAND> command;
                    ThreadProtecetedVar<bool> isnew;

            };

            //command buffers
            CommandBuffer<Pose> poseCommand;
            CommandBuffer<Twist> twistCommand;
            CommandBuffer<GoTo> goToCommand;
            CommandBuffer<SimpleActions> simpleActionsCommand;
            CommandBuffer<ComplexActions> complexActionsCommand;
            CommandBuffer<JointState> jointsCommand;
            

            void addControlMessageType(std::string &buf, const ControlMessageType& type);
            void addTelemetryMessageType(std::string &buf, const TelemetryMessageType& type);

            TransportSharedPtr commandTransport;
            TransportSharedPtr telemetryTransport;

            std::string serializeControlMessageType(const ControlMessageType& type);
            //std::string serializeCurrentPose();


            //buffer of sent telemetry (used for telemetry requests)
            TelemetryBuffer buffers;

            uint32_t logLevel;
            


    };

} // end namespace interaction-library-controlled_robot

