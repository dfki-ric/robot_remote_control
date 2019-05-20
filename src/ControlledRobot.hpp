#pragma once

#include "MessageTypes.hpp"
#include <zmq.hpp>

namespace interaction
{
    class ControlledRobot
    {
        public: 
            ControlledRobot(const std::string &addr);

            interaction::Pose getTargetPose();


        private:

            std::shared_ptr<zmq::context_t> context;
	        std::shared_ptr<zmq::socket_t> socket;


            interaction::Pose targetPose;


    };

} // end namespace interaction-library-controlled_robot

