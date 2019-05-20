#pragma once

#include "MessageTypes.hpp"
#include <zmq.hpp>

namespace interaction
{
    class RobotController
    {
        public: 

            RobotController(const std::string &addr);

            
            int setTargetPose(const interaction::Pose & pose);


        private:

            std::shared_ptr<zmq::context_t> context;
	        std::shared_ptr<zmq::socket_t> socket;


    };

} // end namespace interaction-library-controlled_robot

