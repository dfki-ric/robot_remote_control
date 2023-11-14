#pragma once

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"
#include "UpdateThread/UpdateThread.hpp"
#include "UpdateThread/Timer.hpp"
#include "TelemetryBuffer.hpp"
#include "SimpleBuffer.hpp"
#include "CommandBuffer.hpp"
#include "Statistics.hpp"
#include <map>
#include <string>
#include <memory>
#include <vector>
#include <atomic>
#include <algorithm>
#include <unistd.h>
#include <stdexcept>

namespace robot_remote_control {

class ControlledRobot: public UpdateThread {
    public:
        explicit ControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport, const size_t &buffersize = 10);
        virtual ~ControlledRobot();

        /**
         * @brief threaded update function called by UpdateThread that receives commands
         */
        virtual void update();

        /**
         * @brief setup a periodic heardbeat to detect connection loses on the robot side
         * 
         * @warning When there are multiple connections (e.g. vis zmq) with different heartbeats in rare occations the logner heartbeat is used (connection loss (hight freq) right after the low freq time was send)
         * 
         * @param allowedLatency the maximum latency allowed if the heartbeat arrives late
         * @param callback the callback when no heartbeat message arrived in time
         */
        void setupHeartbeatCallback(const float &allowedLatency, const std::function<void(const float&)> &callback) {
            heartbeatAllowedLatency = allowedLatency;
            heartbeatExpiredCallback = callback;
        }

        bool isConnected() {
            return connected.load();
        }

        /**
         * @brief sleeps until isConnected is true. Only works if setHeartBeatDuration is set or the controlled robot is sending something 
         */
        void waitForConnection() {
            while (!isConnected()){ usleep(100000);}
        }

        /**
         * @brief add a channel to a telemetry type
         * 
         * @param type 
         * @return uint16_t 
         */

        template <class PROTO> uint8_t addChannel(const TelemetryMessageType& type, const std::string name = "") {
            uint8_t channelno = buffers->registerType<PROTO>(type, 1);
            ChannelDefinition* channel = channels.add_channel();
            channel->set_name(name);
            channel->set_messagetype(type);
            channel->set_channelno(channelno);
            sendTelemetry(channels, CHANNELS_DEFINITION, true, 0);
            return channelno;
        }

        // Command Callbacks

        /**
         * @brief add a callback for any command type received
         * 
         * @param function that takes const uint16_t &type (the tpye id from the message receive) as argument (may also be a lamda)
         */
        void addCommandReceivedCallback(const std::function<void(const uint16_t &type)> &function) {
            commandCallbacks.push_back(function);
        }

        /**
         * @brief add a callback triggered when a specific command type is received
         * 
         * @param type the Command type id from the MessageTypes header
         * @param function 
         */
        bool addCommandReceivedCallback(const uint16_t &type, const std::function<void()> &function) {
            if (commandbuffers[type]) {
                commandbuffers[type]->addCommandReceivedCallback(function);
                return true;
            } else {
                printf("%s:%i there is no bufferes comamnd of type %i\n", __PRETTY_FUNCTION__, __LINE__, type);
            }
            return false;
        }

        /**
         * @brief Get the Statistics object, it is only updated with data if this library is compiled with RRC_STATISTICS
         * 
         * @return Statistics& 
         */
        Statistics& getStatistics() {
            return statistics;
        }

        // Command getters

        /**
         * @brief Get the Target Pose the robot should move to
         * 
         * @return true if the command was not read before
         * @param command the last received command
         */
        bool getTargetPoseCommand(Pose *command, bool onlyNewest = true) {
            return poseCommand->read(command, onlyNewest);
        }

        /**
         * @brief Get the Twist Command with velocities to robe should move at
         * 
         * @return true if the command was not read before
         * @param command the last received command
         */
        bool getTwistCommand(Twist *command, bool onlyNewest = true) {
            return twistCommand->read(command, onlyNewest);
        }

        /**
         * @brief Get the GoTo Command the robot should execute
         *
         * @return true if the command was not read before
         * @param command the last received command
         */
        bool getGoToCommand(GoTo *command, bool onlyNewest = true) {
            return goToCommand->read(command, onlyNewest);
        }

        /**
         * @brief Get the Joints Command the robot should execute
         *
         * @return true if the command was not read before
         * @param command the last received command
         */

        bool getJointsCommand(JointCommand *command, bool onlyNewest = true) {
            return jointsCommand->read(command, onlyNewest);
        }


        /**
         * @brief Get the SimpleActions Command the robot should execute
         *
         * @return true if the command was not read before
         * @param command the last received command
         */
        bool getSimpleActionCommand(SimpleAction *command, bool onlyNewest = false) {
            return simpleActionsCommand->read(command, onlyNewest);
        }

        /**
         * @brief Get the ComplexActions Command the robot should execute
         *
         * @return true if the command was not read before
         * @param command the last received command
         */
        bool getComplexActionCommand(ComplexAction *command, bool onlyNewest = false) {
            return complexActionCommandBuffer->read(command, onlyNewest);
        }

        bool getRobotTrajectoryCommand(Poses *command, bool onlyNewest = true) {
            return robotTrajectoryCommand->read(command, onlyNewest);
        }

        bool getCommandRaw(uint16_t type, std::string *dataSerialized, bool onlyNewest = true) {
            return commandbuffers[type]->read(dataSerialized, onlyNewest);
        }

        /**
         * @brief Helper function to get TimeStamp object
         *
         * @return TimeStamp object with current time
         */
        robot_remote_control::TimeStamp getTime();

        // Telemetry setters

    protected:
        /**
         * @brief generic send of telemetry types
         * 
         * @tparam CLASS 
         * @param protodata 
         * @param type 
         * @return int size sent
         */
        template<class CLASS> int sendTelemetry(const CLASS &protodata, const uint16_t& type, bool requestOnly, const uint8_t &channel) {
            if (telemetryTransport.get()) {
                size_t headersize = sizeof(uint16_t)+sizeof(uint8_t);
                std::string buf;
                buf.resize(headersize);
                uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf.data()));
                *data = type;
                uint8_t* chan = reinterpret_cast<uint8_t*>(const_cast<char*>(buf.data() + sizeof(uint16_t)));
                *chan = channel;
                protodata.AppendToString(&buf);
                // store latest data for future requests
                {
                    auto lockedbuffer = buffers->lockedAccess();
                    // check existence only if channel is actially set
                    if (channel > 0 && channel > lockedbuffer.get()[type].size()-1) {
                        throw std::out_of_range ("Channel does not exist");
                    }
                    RingBufferAccess::pushData(lockedbuffer.get()[type][channel], protodata, true);
                }
                if (!requestOnly) {
                    uint32_t bytes = telemetryTransport->send(buf);
                    updateStatistics(bytes, type);
                    return bytes - headersize;
                }
                return buf.size() - headersize;
            }
            printf("ERROR Transport invalid\n");
            return 0;
        }

        int sendTelemetryRaw(const uint16_t& type, const std::string& serialized, const uint8_t &channel = 0) {
            if (telemetryTransport.get()) {
                size_t headersize = sizeof(uint16_t)+sizeof(uint8_t);
                std::string buf;
                buf.resize(headersize);
                uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf.data()));
                *data = type;
                uint8_t* chan = reinterpret_cast<uint8_t*>(const_cast<char*>(buf.data() + sizeof(uint16_t)));
                *chan = channel;
                buf.append(serialized);
                {
                    // check existence only if channel is actially set
                    if (channel > 0 && channel > buffers->lockedAccess().get()[type].size()-1) {
                        throw std::out_of_range ("Channel does not exist");
                    }
                    buffers->pushSerialized(type, buf, channel);
                    // RingBufferAccess::pushData(lockedbuffer.get()[type][channel], protodata, true);
                }
                uint32_t bytes = telemetryTransport->send(buf);
                updateStatistics(bytes, type);
                return bytes - headersize;
            }
            printf("ERROR Transport invalid\n");
            return 0;
        }

        void updateStatistics(const uint32_t &bytesSent, const uint16_t &type);

    public:
        /**
         * @brief The robot uses this method to provide information about its controllable joints
         *
         * @param controllableJoints the controllable joints of the robot as a JointState
         * @return int number of bytes sent
         */
        int initControllableJoints(const JointState& telemetry) {
            return sendTelemetry(telemetry, CONTROLLABLE_JOINTS, true, 0);
        }

        /**
         * @brief The robot uses this method to provide information about its set of simple actions
         *
         * @param simpleActions the simple actions of the robot to report to the controler
         * the state field of the SimpleActions class should be filled with the max value
         * @return int number of bytes sent
         */
        int initSimpleActions(const SimpleActions& telemetry) {
            return sendTelemetry(telemetry, SIMPLE_ACTIONS, true, 0);
        }
        int updateSimpleActions(const SimpleActions& telemetry) {
            return sendTelemetry(telemetry, SIMPLE_ACTIONS, false, 0);
        }

        /**
         * @brief The robot uses this method to provide information about its set of complex actions
         *
         * @param complexActions the complex actions of the robot as a ComplexActions
         * @return int number of bytes sent
         */
        int initComplexActions(const ComplexActions& telemetry) {
            return sendTelemetry(telemetry, COMPLEX_ACTIONS, true, 0);
        }

        /**
         * @brief sets option on how the interface is implemented e.g. id simpleactons are static ot can change
         * 
         * @param options 
         * @return int 
         */
        int initInterfaceOptions(const InterfaceOptions& options) {
            return sendTelemetry(options, INTERFACE_OPTIONS, true , 0);
        }

        /**
         * @brief The robot uses this method to provide information about its sensors
         * The name is only mandatory here, setSimpleSnsor() may omit this value and identify by id
         * 
         * @param telemetry a list of simple sensors and their names/ids, other firelds not nessecary
         * @return int  number of bytes sent
         */
        int initSimpleSensors(const SimpleSensors &telemetry) {
            return sendTelemetry(telemetry, SIMPLE_SENSOR_DEFINITION, true, 0);
        }

        /**
         * @brief The robot uses this method to provide information about its maps
         * The name is only mandatory here, requestMaps() may omit this value and identify by id
         * 
         * @param telemetry a list of simple sensors and their names/ids, other firelds not nessecary
         * @return int  number of bytes sent
         */
        int initMapsDefinition(const MapsDefinition &telemetry) {
            return sendTelemetry(telemetry, MAPS_DEFINITION, true, 0);
        }

        /**
         * @brief The robot uses this method to provide information about its name
         *
         * @param robotName the name of the robot as a RobotName
         * @return int number of bytes sent
         */
        int initRobotName(const RobotName& telemetry) {
            return sendTelemetry(telemetry, ROBOT_NAME, true, 0);
        }

        /**
         * @brief submit the video strem urls
         * 
         * @param telemetry list of streams and camera poses
         * @return int number of bytes sent
         */
        int initVideoStreams(const VideoStreams& telemetry) {
            return sendTelemetry(telemetry, VIDEO_STREAMS, true, 0);
        }

        /**
         * @brief provide cmaera information
         * 
         * @param telemetry 
         * @return int 
         */
        int initCameraInformation(const CameraInformation& telemetry) {
            return sendTelemetry(telemetry, CAMERA_INFORMATION, true, 0);
        }

        /**
         * @brief When more than a single frame can be the target of a pose twist or goto command, this can be used to notify the RobotController
         * which combinationa are possible.
         * 
         * @param telemetry List of frames that can be used to send twist, pose or goto (which target can be set in the header of the command)
         * @return int 
         */
        int initControllableFrames(const ControllableFrames& telemetry) {
            return sendTelemetry(telemetry, CONTROLLABLE_FRAMES, true, 0);
        }

        /**
         * @brief set of fialed that may be downloaded via the rrc lib
         * 
         * @param files name:path list of named files/foilders that can be downloaded
         * @return int 
         */
        int initFiles(const FileDefinition& files) {
            this->files.MergeFrom(files);
            return sendTelemetry(files, FILE_DEFINITION, true, 0);
        }

        int initRobotModel(const FileDefinition& filedef, const std::string& modelfilename = "") {
            RobotModelInformation modelinfo;

            if (modelfilename == "") {
                // single file model
                modelinfo.mutable_filedef()->CopyFrom(filedef);
            } else {
                // folder model
                modelinfo.mutable_filedef()->CopyFrom(filedef);
                modelinfo.set_modelfilename(modelfilename);
            }
            if (!initFiles(filedef)) {
                return -1;
            }
            return sendTelemetry(modelinfo, ROBOT_MODEL_INFORMATION, true, 0);
        }

        std::shared_future<bool> requestPermission(const PermissionRequest &permissionrequest) {
            // get and init promise in map
            std::promise<bool> &promise = pendingPermissionRequests[permissionrequest.requestuid()];
            sendTelemetry(permissionrequest, PERMISSION_REQUEST, false, 0);
            // try {
            return promise.get_future().share();
            // }
            // catch (const std::future_error& e) {
            //     // printf("%s\n", e.what());
            // }
        }

        /**
         * @brief Send a log message, is only send, if the log level set by the controller is higher
         * or equal to the lvl or higher than CUSTOM in these parameters. CUSTOM Messages can be 20 or higher
         * 
         * @param lvl LogLevel (NONE=0,FATAL,ERROR,WARN,INFO,DEBUG,CUSTOM=20)
         * @param message the message to send
         * @return int number of bytes sent
         */
        int setLogMessage(enum LogLevel lvl, const std::string& message);

        /**
         * @brief Set the Log Message object, 
         * 
         * @param log_message 
         * @return int number of bytes sent
         */
        int setLogMessage(const LogMessage& log_message);

        /**
         * @brief Set the Robot State as a single string
         * 
         * @param state the state description
         * @return int number of bytes sent
         */
        int setRobotState(const std::string& state);

        /**
         * @brief Set the Robot State as string vector
         * 
         * @param state 
         * @return int 
         */
        int setRobotState(const std::vector<std::string> state);

        /**
         * @brief Set the Robot State as object
         * 
         * @param state 
         * @return int 
         */
        int setRobotState(const RobotState& state);

        /**
         * @brief Set the current Pose of the robot
         * 
         * @param telemetry current pose
         * @return int number of bytes sent
         */
        int setCurrentPose(const Pose& telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, CURRENT_POSE, false, channel);
        }

        int setCurrentTwist(const Twist& telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, CURRENT_TWIST, false, channel);
        }

        int setCurrentAcceleration(const Acceleration& telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, CURRENT_ACCELERATION, false, channel);
        }

        int setCurrentIMUValues(const IMU &imu, const uint8_t &channel = 0) {
            return sendTelemetry(imu, IMU_VALUES, false, channel);
        }

        int setCurrentContactPoints(const ContactPoints &points, const uint8_t &channel = 0) {
            return sendTelemetry(points, CONTACT_POINTS, false, channel);
        }

        /**
         * @brief Set repeated field of poses
         * 
         * @param telemetry several pose
         * @return int number of bytes sent
         */
        int setPoses(const Poses& telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, POSES, false, channel);
        }

        /**
         * @brief Set the current JointState of the robot
         * 
         * @param telemetry current JointState
         * @return int number of bytes sent
         */
        int setJointState(const JointState& telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, JOINT_STATE, false, channel);
        }

        /**
         * @brief Set the current WrenchState of the robot
         * 
         * @param telemetry current WrenchState
         * @return int number of bytes sent
         */
        int setWrenchState(const WrenchState& telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, WRENCH_STATE, false, channel);
        }

        /**
         * @brief Set a single Simple Sensor value
         * the name strin can be omitted, if it was provided using initSimpleSensors()
         * 
         * @param telemetry a single sensor value
         * @return int number of bytes sent
         */
        int setSimpleSensor(const SimpleSensor &telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, SIMPLE_SENSOR_VALUE, false, channel);
        }

        /**
         * @brief Set the Map object, maps are not sent via telemetry, they have to be requsted 
         *  to be sent via the command channel
         * 
         * @param map 
         * @param mapId defined the map type defiend in MapMessageType
         * @return int 
         */

        int setMap(const Map & map, const uint32_t &mapId) {
            return setMap(map.SerializeAsString(), mapId);
        }

        /**
         * @brief Set the Binary Map object
         * This one is not limited to protobuf types
         * 
         * @param map 
         * @param mapId 
         * @return int 
         */
        int setMap(const std::string & map, const uint32_t &mapId) {
            mapBuffer.initBufferID(mapId);
            RingBufferAccess::pushData(mapBuffer.lockedAccess().get()[mapId], map, true);
            // maps are not sent automatically
            // return sendTelemetry(map, MAP, true);
            return true;
        }

        /**
         * @brief Set the Point Cloud object to be requestes as a map
         * 
         * @param pointcloud 
         * @return int 
         */
        int setPointCloud(const robot_remote_control::PointCloud &pointcloud, const uint8_t &channel = 0) {
            return sendTelemetry(pointcloud, POINTCLOUD, false, channel);
        }


        int setPointCloudMap(const robot_remote_control::PointCloud &pointcloud) {
            robot_remote_control::Map map;
            map.mutable_map()->PackFrom(pointcloud);
            return setMap(map, robot_remote_control::POINTCLOUD_MAP);
        }

        /**
         * @brief Grid map transferredas simplesensor Maps are sent on request, 
         * 
         */
        int setGridMap(const GridMap &gridmap) {
            robot_remote_control::Map map;
            map.mutable_map()->PackFrom(gridmap);
            return setMap(map, robot_remote_control::GRID_MAP);
        }

        /**
         * @brief Set current transforms
         *
         * @param telemetry a repeated field of transforms
         * @return int number of bytes sent
         */
        int setCurrentTransforms(const Transforms &telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, TRANSFORMS, false, channel);
        }

        /**
         * @brief Set a Image identifiable by the frame in the header
         * 
         * @param telemetry 
         * @return int 
         */
        int setImage(const Image &telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, IMAGE, false, channel);
        }

        /**
         * @brief Set a Image with multiple Layers
         * 
         * @param telemetry 
         * @return int 
         */
        int setImageLayers(const ImageLayers &telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, IMAGE_LAYERS, false, channel);
        }

        /**
         * @brief Set the Odometry object
         * 
         * @param telemetry 
         * @return int 
         */
        int setOdometry(const Odometry &telemetry, const uint8_t &channel = 0) {
            return sendTelemetry(telemetry, ODOMETRY, false, channel);
        }


    protected:
        virtual ControlMessageType receiveRequest();

        virtual ControlMessageType evaluateRequest(const std::string& request);

        bool loadFile(File* file, const std::string &path, bool compressed = false);

        bool loadFolder(Folder* folder, const std::string &path, bool compressed = false);

        void notifyCommandCallbacks(const uint16_t &type);

        virtual ControlMessageType handleTelemetryRequest(const std::string& serializedMessage, TransportSharedPtr commandTransport);
        virtual ControlMessageType handleMapRequest(const std::string& serializedMessage, TransportSharedPtr commandTransport);
        virtual ControlMessageType handlePermissionRequest(const std::string& serializedMessage, TransportSharedPtr commandTransport);
        virtual ControlMessageType handleFileRequest(const std::string& serializedMessage, TransportSharedPtr commandTransport);
        virtual ControlMessageType handleCommandRequest(const ControlMessageType &msgtype, const std::string& serializedMessage, robot_remote_control::TransportSharedPtr commandTransport);


        // command buffers
        std::unique_ptr<CommandBuffer<Pose>> poseCommand;
        std::unique_ptr<CommandBuffer<Twist>> twistCommand;
        std::unique_ptr<CommandBuffer<GoTo>> goToCommand;
        std::unique_ptr<CommandBuffer<SimpleAction>> simpleActionsCommand;
        std::unique_ptr<CommandBuffer<ComplexAction>> complexActionCommandBuffer;
        std::unique_ptr<CommandBuffer<JointCommand>> jointsCommand;
        std::unique_ptr<CommandBuffer<HeartBeat>> heartbeatCommand;
        std::unique_ptr<CommandBuffer<Permission>> permissionCommand;
        std::unique_ptr<CommandBuffer<Poses>> robotTrajectoryCommand;

        std::vector< std::function<void(const uint16_t &type)> > commandCallbacks;

        FileDefinition files;
        HeartBeat heartbeatValues;
        Timer heartbeatTimer;
        float heartbeatAllowedLatency;
        std::function<void(const float&)> heartbeatExpiredCallback;
        std::atomic<bool> connected;

        SimpleBuffer<std::string> mapBuffer;

        std::array<CommandBufferBase*, CONTROL_MESSAGE_TYPE_NUMBER> commandbuffers;
        void registerCommandType(const uint32_t & ID, CommandBufferBase *bufptr) {
            commandbuffers[ID] = bufptr;
        }

        void addControlMessageType(std::string *buf, const ControlMessageType& type);
        void addTelemetryMessageType(std::string *buf, const TelemetryMessageType& type);

        TransportSharedPtr commandTransport;
        TransportSharedPtr telemetryTransport;

        std::string serializeControlMessageType(const ControlMessageType& type);
        // std::string serializeCurrentPose();

        template <class PROTO> void registerTelemetryType(const uint16_t &type) {
            buffers->registerType<PROTO>(type, 1);
            #ifdef RRC_STATISTICS
                PROTO telemetry_type;
                statistics.names[type] = telemetry_type.GetTypeName();
            #endif
        }
        // buffer of sent telemetry (used for telemetry requests)
        std::shared_ptr<TelemetryBuffer> buffers;

        uint32_t logLevel;

        std::map<std::string, std::promise<bool> > pendingPermissionRequests;

        Statistics statistics;
        ChannelsDefinition channels;

        Transport::Flags receiveflags;

};

}  // namespace robot_remote_control

