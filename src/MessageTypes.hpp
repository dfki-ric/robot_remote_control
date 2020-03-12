#pragma once

#include "Types/RobotRemoteControl.pb.h"

namespace robot_remote_control
{   
    /**
     * @brief The data channel type
     * 
     */
    enum ControlMessageType{NO_CONTROL_DATA=0,
                            TARGET_POSE_COMMAND,    //target Pose the robot should move to
                            LEFT_ARM_END_EFFECTOR_TARGET_POSE_COMMAND,
                            TWIST_COMMAND,           //directly moce the robot Base
                            LEFT_ARM_END_EFFECTOR_TWIST_COMMAND,
                            JOINTS_COMMAND,
                            SIMPLE_ACTIONS_COMMAND,
                            COMPLEX_ACTION_COMMAND,
                            GOTO_COMMAND,
                            TELEMETRY_REQUEST,
                            LOG_LEVEL_SELECT,
                            CONTROL_MESSAGE_TYPE_NUMBER
                            };

    enum TelemetryMessageType{  NO_TELEMETRY_DATA=0,
                                CURRENT_POSE,       //the curretn Pose of the robot base
                                JOINT_STATE,         //current Joint values
                                CONTROLLABLE_JOINTS, // info about the controllable joints of the robot
                                SIMPLE_ACTIONS, // info about the simple actions of the robot
                                COMPLEX_ACTIONS, // info about the complex actions of the robot
                                ROBOT_NAME, // the name of the robot
                                ROBOT_STATE, //robot state description (string)
                                LOG_MESSAGE, //log messages,
                                VIDEO_STREAMS, //definition of video streams
                                SIMPLE_SENSOR_DEFINITION,
                                SIMPLE_SENSOR_VALUE,
                                TELEMETRY_MESSAGE_TYPES_NUMBER //LAST element
                            };

    /**
     * @brief 
     * 
     * use Custom+x for curtom messages
     */
    enum LogLevel{NONE=0,FATAL,ERROR,WARN,INFO,DEBUG,CUSTOM=20};

//http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html

} // end namespace interaction-library-controlled_robot




