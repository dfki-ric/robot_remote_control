#pragma once

#include "Types/controlledRobot.pb.h"

namespace controlledRobot
{   
    /**
     * @brief The data channel type
     * 
     */
    enum ControlMessageType{NO_DATA=0,
                            TARGET_POSE_COMMAND,    //target Pose the robot should move to
                            TWIST_COMMAND,           //directly moce the robot Base
                            JOINTS_COMMAND,
                            ACTIONS_COMMAND,
                            GOTO_COMMAND,
                            TELEMETRY_REQUEST
                            };

    enum TelemetryMessageType{  NO_TELEMETRY_DATA=0,
                                CURRENT_POSE,       //the curretn Pose of the robot base
                                JOINT_STATE,         //current Joint values
                                CONTROLLABLE_JOINTS, // info about the controllable joints of the robot
                                SIMPLE_ACTIONS, // info about the simple actions of the robot
                                COMPLEX_ACTIONS, // info about the complex actions of the robot
                                TELEMETRY_MESSAGE_TYPES_NUMBER //LAST element
                            };


//http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html

} // end namespace interaction-library-controlled_robot




