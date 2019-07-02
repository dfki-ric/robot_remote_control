#pragma once

#include <interaction.pb.h>

namespace interaction
{   
    /**
     * @brief The data channel type
     * 
     */
    enum ControlMessageType{NO_DATA=0,
                            TARGET_POSE_COMMAND,    //target Pose the robot should move to
                            TWIST_COMMAND           //directly moce the robot Base
                            };

    enum TelemetryMessageType{  NO_TELEMETRY_DATA=0,
                                CURRENT_POSE,       //the curretn Pose of the robot base
                                JOINT_STATE,         //current Joint values
                                TELEMETRY_MESSAGE_TYPES_NUMBER //LAST element
                            };

//http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html

} // end namespace interaction-library-controlled_robot







