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
             * @return Pose the target pose
             * @TODO isNEW?
             */
            Pose getTargetPose(){
                return targetPose.get();
            }

            /**
             * @brief Get the Twist Command with velocities to robe should move at
             * 
             * @return Twist 
             */
            std::pair<unsigned long long int, Twist> getTwistCommand(){
                unsigned long long int counter = twistCommandGetCounter.get();
                twistCommandGetCounter.set(counter + 1);
                return std::pair<unsigned long long int, Twist>(counter, twistCommand.get());
            }

            /**
             * @brief Get the GoTo Command the robot should execute
             *
             * @return std::pair<unsigned long long int, GoTo>
             */
            std::pair<long long int, GoTo> getGoToCommand() {
                long long int counter = goToCommandGetCounter.get();
                if (counter != -1) goToCommandGetCounter.set(counter + 1);
                return std::pair<long long int, GoTo>(counter, goToCommand.get());
            }

            /**
             * @brief Get the Joints Command the robot should execute
             *
             * @return std::pair<unsigned long long int, JointState>
             */
            std::pair<unsigned long long int, JointState> getJointsCommand() {
                unsigned long long int counter = jointsCommandGetCounter.get();
                jointsCommandGetCounter.set(counter + 1);
                return std::pair<unsigned long long int, JointState>(counter, jointsCommand.get());
            }


            /**
             * @brief Get the SimpleActions Command the robot should execute
             *
             * @return std::pair<unsigned long long int, SimpleActions>
             */
            std::pair<unsigned long long int, SimpleActions> getSimpleActionsCommand() {
                unsigned long long int counter = simpleActionsCommandGetCounter.get();
                simpleActionsCommandGetCounter.set(counter + 1);
                return std::pair<unsigned long long int, SimpleActions>(counter, simpleActionsCommand.get());
            }

            /**
             * @brief Get the ComplexActions Command the robot should execute
             *
             * @return std::pair<unsigned long long int, ComplexActions>
             */
            std::pair<unsigned long long int, ComplexActions> getComplexActionsCommand() {
                unsigned long long int counter = complexActionsCommandGetCounter.get();
                complexActionsCommandGetCounter.set(counter + 1);
                return std::pair<unsigned long long int, ComplexActions>(counter, complexActionsCommand.get());
            }

            // Telemetry setters

            /**
             * @brief Set the current Pose of the robot
             * 
             * @param pose current pose
             * @return int number of bytes sent
             */
            int setCurrentPose(const Pose& pose);

            /**
             * @brief Set the curretn JointState of the robot
             * 
             * @param pose current JointState
             * @return int number of bytes sent
             */
            int setJointState(const JointState& state);

            /**
             * @brief The robot uses this method to provide information about its controllable joints
             *
             * @param controllableJoints the controllable joints of the robot as a JointState
             * @return int number of bytes sent
             */
            int setControllableJoints(const JointState& controllableJoints);

            /**
             * @brief The robot uses this method to provide information about its set of simple actions
             *
             * @param simpleActions the simple actions of the robot as a SimpleActions
             * @return int number of bytes sent
             */
            int setSimpleActions(const SimpleActions& simpleActions);

            /**
             * @brief The robot uses this method to provide information about its set of complex actions
             *
             * @param complexActions the complex actions of the robot as a ComplexActions
             * @return int number of bytes sent
             */
            int setComplexActions(const ComplexActions& complexActions);

            /**
             * @brief The robot uses this method to provide information about its name
             *
             * @param robotName the name of the robot as a RobotName
             * @return int number of bytes sent
             */
            int setRobotName(const RobotName& robotName);


            int setRobotState(const std::string& state);

            int setLogMessage(enum LogLevel lvl, const std::string& message);

            int setLogMessage(const LogMessage& log_message);



        protected:
            virtual ControlMessageType receiveRequest();

            virtual ControlMessageType evaluateRequest(const std::string& request);

        private:
            void addControlMessageType(std::string &buf, const ControlMessageType& type);
            void addTelemetryMessageType(std::string &buf, const TelemetryMessageType& type);

            TransportSharedPtr commandTransport;
            TransportSharedPtr telemetryTransport;

            std::string serializeControlMessageType(const ControlMessageType& type);
            //std::string serializeCurrentPose();

            //buffers
            ThreadProtecetedVar<Pose> targetPose;
            // ThreadProtecetedVar<Pose> currentPose;
            ThreadProtecetedVar<Twist> twistCommand;
            ThreadProtecetedVar<unsigned long long int> twistCommandGetCounter;
            ThreadProtecetedVar<GoTo> goToCommand;
            ThreadProtecetedVar<long long int> goToCommandGetCounter;
            ThreadProtecetedVar<SimpleActions> simpleActionsCommand;
            ThreadProtecetedVar<unsigned long long int> simpleActionsCommandGetCounter;
            ThreadProtecetedVar<ComplexActions> complexActionsCommand;
            ThreadProtecetedVar<unsigned long long int> complexActionsCommandGetCounter;
            ThreadProtecetedVar<JointState> jointsCommand;
            ThreadProtecetedVar<unsigned long long int> jointsCommandGetCounter;


            TelemetryBuffer buffers;

            uint32_t logLevel;


            /**
             * @brief 
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
            


    };

} // end namespace interaction-library-controlled_robot

