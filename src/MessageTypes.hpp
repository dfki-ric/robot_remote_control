#pragma once

#include "Types/RobotRemoteControl.pb.h"

namespace robot_remote_control {

    /**
     * @brief The data channel type
     * 
     */
    enum ControlMessageType : uint16_t { NO_CONTROL_DATA = 0,
                            TARGET_POSE_COMMAND,     // target Pose the robot should move to
                            TWIST_COMMAND,           // directly move the robot Base
                            JOINTS_COMMAND,
                            SIMPLE_ACTIONS_COMMAND,
                            COMPLEX_ACTION_COMMAND,
                            GOTO_COMMAND,
                            TELEMETRY_REQUEST,
                            LOG_LEVEL_SELECT,
                            MAP_REQUEST,
                            HEARTBEAT,
                            PERMISSION,
                            ROBOT_TRAJECTORY_COMMAND,
                            CONTROL_MESSAGE_TYPE_NUMBER  // LAST element
                            };

    enum TelemetryMessageType : uint16_t { NO_TELEMETRY_DATA = 0,
                                CURRENT_POSE,               // the current Pose of the robot base
                                JOINT_STATE,                // current Joint values
                                CONTROLLABLE_JOINTS,        // info about the controllable joints of the robot
                                SIMPLE_ACTIONS,             // info about the simple actions of the robot
                                COMPLEX_ACTIONS,            // info about the complex actions of the robot
                                ROBOT_NAME,                 // the name of the robot
                                ROBOT_STATE,                // robot state description (string)
                                LOG_MESSAGE,                // log messages
                                VIDEO_STREAMS,              // definition of video streams
                                SIMPLE_SENSOR_DEFINITION,   // definition of available simple sensors
                                SIMPLE_SENSOR_VALUE,        // actual simple sensor values
                                WRENCH_STATE,               // current Wrench values
                                MAPS_DEFINITION,
                                MAP,
                                POSES,                      // number of poses
                                TRANSFORMS,                 // number of transforms
                                PERMISSION_REQUEST,         // a request for a permission
                                POINTCLOUD,                 // Pointcloud as telemetry
                                IMU_VALUES,                 // raw IMU
                                CONTACT_POINTS,             // contact points
                                CURRENT_TWIST,              // the current movement speeds of the robot
                                CURRENT_ACCELERATION,       // the current movement accelerations of the robot
                                CAMERA_INFORMATION,         // information on cameras (identified by frame in header)
                                IMAGE,                      // raw images, in case video streasm are not an option (identified by frame in header)
                                IMAGE_LAYERS,               // Collection of images with a single timestamp (e.g. stereo pairs)
                                TELEMETRY_MESSAGE_TYPES_NUMBER  // LAST element
                            };

    // enum MapMessageType : uint16_t {  
    //     NO_MAP_DATA = 0,
    //     POINTCLOUD_MAP,
    //     MAP_MESSAGE_TYPES_NUMBER  // LAST element

    // };

    /**
     * @brief 
     * 
     * use Custom+x for custom messages
     */
    enum LogLevel : uint16_t {NONE = 0, FATAL, ERROR, WARN, INFO, DEBUG, CUSTOM = 20};

}  // end namespace robot_remote_control


