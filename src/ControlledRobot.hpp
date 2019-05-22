#pragma once

#include "MessageTypes.hpp"
#include <zmq.hpp>

namespace interaction
{
    class ControlledRobot
    {
        public: 
            ControlledRobot(const std::string &addr);
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
            zmq::message_t serializeControlMessageType(const ControlMessageType& type);
            zmq::message_t serializeCurrentPose();

            std::shared_ptr<zmq::context_t> context;
	        std::shared_ptr<zmq::socket_t> socket;


            interaction::Pose targetPose;
            interaction::Pose currentPose;
            interaction::Twist twistCommand;



    };

} // end namespace interaction-library-controlled_robot

