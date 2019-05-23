#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"

namespace interaction
{
    class ControlledRobot
    {
        public: 

            ControlledRobot(std::shared_ptr<interaction::Transport> commandTransport,std::shared_ptr<interaction::Transport> telemetryTransport);
            virtual ~ControlledRobot(){};

            void update();

            interaction::Pose getTargetPose(){
                return targetPose;
            }

            void setCurrentPose(const interaction::Pose& pose){
                currentPose = pose;
            }

            interaction::Twist getTwistCommand(){
                return twistCommand;
            }

        protected:
            virtual ControlMessageType receiveRequest();

            ControlMessageType evaluateRequest(const std::string& request);

        private:
            void addControlMessageType(std::string &buf, const ControlMessageType& type);

            std::shared_ptr<interaction::Transport> commandTransport;
            std::shared_ptr<interaction::Transport> telemetryTransport;

            std::string serializeControlMessageType(const ControlMessageType& type);
            std::string serializeCurrentPose();



            interaction::Pose targetPose;
            interaction::Pose currentPose;
            interaction::Twist twistCommand;



    };

} // end namespace interaction-library-controlled_robot

