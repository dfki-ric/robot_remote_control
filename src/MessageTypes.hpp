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
                            JOINT_NAME_REQUEST,
                            ACTIONS_COMMAND,
                            ACTION_NAME_REQUEST
                            };

    enum TelemetryMessageType{  NO_TELEMETRY_DATA=0,
                                CURRENT_POSE,       //the curretn Pose of the robot base
                                JOINT_STATE,         //current Joint values
                                TELEMETRY_MESSAGE_TYPES_NUMBER //LAST element
                            };

//http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html

} // end namespace interaction-library-controlled_robot




