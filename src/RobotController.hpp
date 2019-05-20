#pragma once

#include "MessageTypes.hpp"
#include <zmq.hpp>

namespace interaction
{
    class RobotController
    {
        public: 

            RobotController(const std::string &addr);
            virtual ~RobotController(){};
            
            int setTargetPose(const interaction::Pose & pose);
            
            interaction::Pose getCurrentPose();

        protected:
            virtual int sendRequest(const std::string& serializedMessage);

            int evaluateReply(const std::string& reply);

        private:

            std::shared_ptr<zmq::context_t> context;
	        std::shared_ptr<zmq::socket_t> socket;

            interaction::Pose currentPose;


    };

} // end namespace interaction-library-controlled_robot

