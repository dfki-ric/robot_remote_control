#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"
#include "UpdateThread/UpdateThread.hpp"


namespace controlledRobot
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
             * @return controlledRobot::Pose the target pose
             * @TODO isNEW?
             */
            controlledRobot::Pose getTargetPose(){
                return targetPose.get();
            }

            /**
             * @brief Get the Twist Command with velocities to robe should move at
             * 
             * @return controlledRobot::Twist 
             */
            controlledRobot::Twist getTwistCommand(){
                return twistCommand.get();
            }

            /**
             * @brief Get the GoTo Command the robot should execute
             *
             * @return controlledRobot::GoTo
             */
            std::pair<unsigned long long int, controlledRobot::GoTo> getGoToCommand() {
                return std::pair<unsigned long long int, controlledRobot::GoTo>(goToCommandGetCounter++, goToCommand.get());
            }


            // Telemetry setters

            /**
             * @brief Set the current Pose of the robot
             * 
             * @param pose current pose
             * @return int number of bytes sent
             */
            int setCurrentPose(const controlledRobot::Pose& pose);

            /**
             * @brief Set the curretn JointState of the robot
             * 
             * @param pose current JointState
             * @return int number of bytes sent
             */
            int setJointState(const controlledRobot::JointState& state);




        protected:
            virtual ControlMessageType receiveRequest();

            virtual ControlMessageType evaluateRequest(const std::string& request);

        private:
            void addControlMessageType(std::string &buf, const ControlMessageType& type);
            void addTelemetryMessageType(std::string &buf, const TelemetryMessageType& type);

            TransportSharedPtr commandTransport;
            TransportSharedPtr telemetryTransport;

            std::string serializeControlMessageType(const ControlMessageType& type);
            std::string serializeCurrentPose();

            //buffers
            ThreadProtecetedVar<controlledRobot::Pose> targetPose;
            ThreadProtecetedVar<controlledRobot::Pose> currentPose;
            ThreadProtecetedVar<controlledRobot::Twist> twistCommand;
            ThreadProtecetedVar<controlledRobot::GoTo> goToCommand;
            unsigned long long int goToCommandGetCounter;

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
                    return telemetryTransport->send(buf) - sizeof(uint16_t);
                }
                printf("ERROR Transport invalid\n");
                return 0;
            };
            


    };

} // end namespace interaction-library-controlled_robot

