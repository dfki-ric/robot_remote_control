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

            interaction::Pose getTargetPose();

            void setCurrentPose(const interaction::Pose& pose){
                currentPose = pose;
            }

        protected:
            virtual int receiveRequest();

            int evaluateRequest(const std::string& request);

        private:

            zmq::message_t serializeCurrentPose();

            std::shared_ptr<zmq::context_t> context;
	        std::shared_ptr<zmq::socket_t> socket;


            interaction::Pose targetPose;
            interaction::Pose currentPose;


    };

} // end namespace interaction-library-controlled_robot

