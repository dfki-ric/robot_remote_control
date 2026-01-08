#pragma once

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <mutex>
#include <atomic>
#include <experimental/filesystem>
#include <fstream>
#include <unistd.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/util/json_util.h>

#include "Types/RobotRemoteControl.pb.h"
#include "Transports/Transport.hpp"
#include "TelemetryBuffer.hpp"
#include "Statistics.hpp"
#include "UpdateThread/UpdateThread.hpp"
#include "UpdateThread/Timer.hpp"



namespace robot_remote_control {

class RobotController: public UpdateThread {
    public:
        explicit RobotController(TransportSharedPtr commandTransport,
                                TransportSharedPtr telemetryTransport = TransportSharedPtr(),
                                const size_t &buffersize = 10,
                                const float &maxLatency = 1);
        virtual ~RobotController();

        enum SerializationMode {BINARY, JSON};
        void setSerializationMode(const SerializationMode & mode) {
            serializationMode = mode;
        }
        SerializationMode getSerializationMode() {
            return serializationMode;
        }

        /**
         * @brief in case there is a telemetry connection, receive all and fill the data fields
         */
        virtual void update();

        /**
         * @brief Set the overwrite mode of a specific buffers (default false)
         * 
         * @param type TelemetryMessageType enum
         * @param overwrite if false: new data is dropped if the buffer is full, if true: oldest data in buffer is overwritten
         */
        bool setSingleTelemetryBufferOverwrite(TelemetryMessageType type, bool overwrite = true, const ChannelId &channel = 0);

        /**
         * @brief Set the overwrite mode of all buffers (default false)
         * 
         * @param overwrite if false: new data is dropped if the buffer is full, if true: oldest data in buffer is overwritten
         */
        void setTelemetryBufferOverwrite(bool overwrite = true, const ChannelId &channel = 0);

        /**
         * @brief Set the buffer size of a sinlgle telemetry type (default value set in RobotController() constructor)
         * 
         * @warning buffer will be emptied on resize
         * 
         * @param type TelemetryMessageType enum
         * @param newsize the new size of the buffer 
         */
        bool setSingleTelemetryBufferSize(TelemetryMessageType type, size_t newsize = 10, const ChannelId &channel = 0);

        bool addChannelBuffer(const TelemetryMessageType& type, const ChannelId &channel, const size_t &buffersize = 10) {
            return buffers->addChannelBuffer(type, channel, buffersize);
        }

        /**
         * @brief sets the expected next heartbeat time on the robot side
         * The value is trasmitted with the heartbeat message and is evaluated on the robot side, (stable) latency
         * it not an issue
         */
        void setHeartBeatDuration(const float &duration_seconds) {
            heartBeatDuration = duration_seconds;
            heartBeatTimer.start(heartBeatDuration);
        }
        /**
         * @brief Set the allowed maximum latency for receiving heartbeat reply messages after sending a heartbeat
         * the default is set in the constructor
         */
        void setMaxLatency(const float &value) {
            maxLatency = value;
        }

        /**
         * @brief can be used to override the default printout when the connection timed out
         * 
         * @param callback 
         */
        void setupLostConnectionCallback(const std::function<void(const float&)> &callback) {
            printf("warning: setupLostConnectionCallback deprecated, use setupDisconnectedCallback()\n");
            setupDisconnectedCallback(callback);
        }
        void setupDisconnectedCallback(const std::function<void(const float&)> &callback) {
            lostConnectionCallback = callback;
        }

        void setupConnectedCallback(const std::function<void()> &callback) {
            connectedCallback=callback;
        }

        /**
         * @brief return then current state of the connection using the heartbeat.
         * @return the current status of the connection, if a hearbeat duration is set using setHeartBeatDuration()
         * 
         */
        bool isConnected() {
            return connected.load();
        }

        /**
         * @brief sleeps until isConnected is true. Only works if setHeartBeatDuration is set or the controlled robot is sending something 
         */
        void waitForConnection() {
            while (! isConnected()) {usleep(100000);}
        }

        /**
         * @brief Check if the Robot has exactly! the same protocol version. Even if not, this library has a high probability to work as intended
         * due to protobufs forward/backward compatibility features, as long the mayor version is the same
         * 
         * @return true the MessageTypes.hpp and Types/RobotRemoteControl.proto files of the Robos match exacly the local ones
         * @return false there are diffences, but as long the requestLibraryVersion() has the same value everythin should still work
         */
        bool checkProtocolVersion();

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string requestProtocolVersion();

        std::string protocolVersion();

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool checkLibraryVersion();

        /**
         * @brief Get the Library Version 
         * 
         * @return std::string 
         */
        std::string requestLibraryVersion();

        std::string libraryVersion();

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool checkGitVersion();

        /**
         * @brief Get the Library Version 
         * 
         * @return std::string 
         */
        std::string requestGitVersion();

        std::string gitVersion();


        /**
         * @brief Get the Heart Beat Round Trip Time
         * 
         * @return float time in seconds needed to send/receive the last heartbeat (if used)
         */
        float getHeartBeatRoundTripTime() {
            return heartBeatRoundTripTime.load();
        }

        /**
         * @brief Get the number of objects in the buffer
         * 
         * @param type 
         * @return uint32_t 
         */
        uint32_t getTelemetryBufferDataSize(const TelemetryMessageType &type, const ChannelId &channel = 0);

        /**
         * @brief Get the messages dropped of a specific because of full buffer
         * 
         * @param type 
         * @return uint32_t 
         */
        size_t getDroppedTelemetry(const TelemetryMessageType &type, const ChannelId &channel = 0);

        /**
         * @brief Set the Target Pose of the ControlledRobot
         * 
         * @param pose The Pose the robot should move to
         */
        void setTargetPose(const Pose & pose);

        /**
         * @brief Set the Twist Command for direct remote control based on velocities
         * 
         * @param twistCommand 
         */
        void setTwistCommand(const Twist &twistCommand);

        /**
         * @brief Set the GoTo Command of the ControlledRobot
         *
         * @param goToCommand
         */
        void setGoToCommand(const GoTo &goToCommand);

        /**
         * @brief Set the Joint Command object
         * 
         * @param jointsCommand 
         */
        void setJointCommand(const JointCommand &jointsCommand);

        /**
         * @brief Set the SimpleActions command that the controlled robot should execute
         *
         * @param simpleActionsCommand
         */
        void setSimpleActionCommand(const SimpleAction &simpleActionCommand);

        /**
         * @brief Set the ComplexActions command that the controlled robot should execute
         *
         * @param complexActionsCommand
         */
        void setComplexActionCommand(const ComplexAction &complexActionCommand);


        void setRobotTrajectoryCommand(const Poses &robotTrajectoryCommand);


        /**
         * @brief Set the LogLevel of the controlled robot
         * 
         * @param level the desired log level
         */
        void setLogLevel(const LogLevelId &level);

        /**
         * @brief Set the Permission object
         * 
         * @param permission 
         * @return true 
         * @return false 
         */
        bool setPermission(const Permission& permission);


        /**
         * @brief Get the last sent Pose of the robot
         * 
         * @param pose the pose to write the data to
         * @return bool true if new data was read
         */
        bool getCurrentPose(Pose *pose, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(CURRENT_POSE, pose, onlyNewest, channel);
        }

        /**
         * @brief Get the Current Twist object
         * 
         * @param telemetry the Twist object to write to
         * @return true if new data was read
         * @return false otherwise
         */
        bool getCurrentTwist(Twist *telemetry, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(CURRENT_TWIST, telemetry, onlyNewest, channel);
        }

        /**
         * @brief Get the Current Acceleration object
         *
         * @param telemetry the Acceleration object to write to
         * @return true if new data was read
         * @return false otherwise
         */
        bool getCurrentAcceleration(Acceleration *telemetry, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(CURRENT_ACCELERATION, telemetry, onlyNewest, channel);
        }

        /**
         * @brief Get an array of Poses
         * 
         * @param repeated field of poses to write the data to
         * @return bool true if new data was read
         */
        bool getPoses(Poses *poses, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(POSES, poses, onlyNewest, channel);
        }

        /**
         * @brief Get the last sent joint state of the robot
         * 
         * @param jointState the JointState to write the data to
         * @return bool true if new data was read
         */
        bool getCurrentJointState(JointState *jointState, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(JOINT_STATE, jointState, onlyNewest, channel);
        }

        /**
         * Get the last sent wrench state of the robot
         * 
         * @param wrenchState the WrenchState to write the data to
         * @return bool true if new data was read
         */
        bool getCurrentWrenchState(WrenchState* wrenchState, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(WRENCH_STATE, wrenchState, onlyNewest, channel);
        }

        int getCurrentIMUState(IMU* imu, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(IMU_VALUES, imu, onlyNewest, channel);
        }

        int getCurrentContactPoints(ContactPoints* points, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(CONTACT_POINTS, points, onlyNewest, channel);
        }

        /**
         * @brief Get a Simple Sensor object
         * 
         * @param simplesensor SimpleSensor to write data to
         * @return true 
         * @return false 
         */
        int getSimpleSensor(SimpleSensor* telemetry, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(SIMPLE_SENSOR, telemetry, onlyNewest, channel);
        }

        bool getPermissionRequest(PermissionRequest* request) {
            return getTelemetry(PERMISSION_REQUEST, request, false, 0);
        }

        /**
         * @brief Get the Log Message object
         * 
         * @param msg LogMessage to write the data to
         * @return bool true if new data was read
         */
        bool getLogMessage(LogMessage *msg) {
            return getTelemetry(LOG_MESSAGE, msg, false, 0);
        }

        /**
         * @brief Get the Robot State
         * 
         * @param state the string to write the state to
         * @return bool true if new data was read
         */
        bool getRobotState(std::vector<std::string> *state) {
            RobotState protostate;
            int statesleft = getTelemetry(ROBOT_STATE, &protostate, false, 0);
            state->clear();
            for (const std::string &line : protostate.state()) {
                state->push_back(line);
            }
            return statesleft;
        }

        bool getRobotState(RobotState *state) {
            return getTelemetry(ROBOT_STATE, state, false, 0);
        }



        /**
         * @brief Get the current transforms
         *
         * @param Transforms object to write the transforms to
         * @return bool true if new data was read
         */
        bool getCurrentTransforms(Transforms *transforms, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(TRANSFORMS, transforms, onlyNewest, channel);
        }

        /**
         * @brief Get the Point Cloud object
         * 
         * @param pointcloud 
         * @return true 
         * @return false 
         */
        bool getPointCloud(PointCloud *pointcloud, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(POINTCLOUD, pointcloud, onlyNewest, channel);
        }

        /**
         * @brief Get the Point Cloud object
         * 
         * @param laserscan 
         * @return true 
         * @return false 
         */
        bool getLaserScan(LaserScan *laserscan, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(LASER_SCAN, laserscan, onlyNewest, channel);
        }


        bool getImage(Image *image, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(IMAGE, image, onlyNewest, channel);
        }

        bool getImageLayers(ImageLayers *imagelayers, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(IMAGE_LAYERS, imagelayers, onlyNewest, channel);
        }

        bool getOdometry(Odometry* telemetry, bool onlyNewest = false, const ChannelId &channel = 0) {
            return getTelemetry(ODOMETRY, telemetry, onlyNewest, channel);
        }

        /**
         * @brief Some ControlledRobot instaces are using Channels on a single Message Types
         *  e.g. having multiple Point clouds, whiel normally it should be ok to identify those by
         * setting the originating frame, in some applications, you'll need seperate receive buffers,
         * Especially on high cpu load with a mix of high and low frequency of the data.
         * 
         * @param channels 
         */
        bool requestChannelsDefinition(ChannelsDefinition *channels) {
            bool result = requestTelemetry(CHANNELS_DEFINITION, channels, 0);
            for (auto& channeldef : channels->channel()) {
                // save max channel no
                if (channeldef.channelno() > messageChannels[channeldef.messagetype()]) {
                    messageChannels[channeldef.messagetype()] = channeldef.channelno();
                }
                if (channeldef.channelno() >= messageChannelNames[channeldef.messagetype()].size()) {
                    messageChannelNames[channeldef.messagetype()].resize(channeldef.channelno()+1);
                }
                // save name
                messageChannelNames[channeldef.messagetype()][channeldef.channelno()] = channeldef.name();
                messageChannelIdByName[channeldef.messagetype()][channeldef.name()] = channeldef.channelno();
            }
            return result;
        }

        bool requestChannelsDefinition() {
            ChannelsDefinition channels;
            return requestChannelsDefinition(&channels);
        }

        ChannelId getMaxChannelNo(const MessageId& MessageId) {
            return messageChannels[MessageId];
        }

        ChannelId getChannelIdByName(const MessageId& MessageId, const std::string &name) {
            return messageChannelIdByName[MessageId][name];

        }

        std::string getChannelName(const MessageId& MessageId, const ChannelId &channel) {
            if (messageChannelNames[MessageId].size() < channel) {
                return messageChannelNames[MessageId][channel];
            }
            return "";
        }

        /**
         * @brief request the curretn state instead of waiting for the first telemetry message
         * 
         * @param state the string to write the state to
         */
        bool requestRobotState(std::vector<std::string> *state) {
            RobotState protostate;
            bool result = requestTelemetry(ROBOT_STATE, &protostate, 0);
            state->clear();
            for (const std::string &line : protostate.state()) {
                state->push_back(line);
            }
            return result;
        }

        bool requestRobotState(RobotState *state) {
            return requestTelemetry(ROBOT_STATE, state, 0);
        }

        /**
         * @brief Request information about the complex actions of the robot.
         * 
         * @param complexActions where to write the data to
         * @return void
         */
        bool requestComplexActions(ComplexActions *complexActions) {
            return requestTelemetry(COMPLEX_ACTIONS, complexActions, 0);
        }

        /**
         * @brief request options on how the ControlledRobot has to be used
         * 
         * @param options 
         * @return true 
         * @return false 
         */
        bool requestInterfaceOptions(InterfaceOptions *options) {
            return requestTelemetry(INTERFACE_OPTIONS, options, 0);
        }

        /**
         * @brief Request information about the simple actions of the robot.
         * 
         * @param simpleActions where to write the data to
         * @return void
         */
        bool requestSimpleActions(SimpleActions *simpleActions) {
            return requestTelemetry(SIMPLE_ACTIONS, simpleActions, 0);
        }
        bool getSimpleActions(SimpleActions *simpleActions) {
            return getTelemetry(SIMPLE_ACTIONS, simpleActions, true, 0);
        }

        /**
         * @brief Request information about the controllable joints of the robot.
         * 
         * @param jointState where to write the data to
         * @return void
         */
        bool requestControllableJoints(JointState *jointState) {
            return requestTelemetry(CONTROLLABLE_JOINTS, jointState, 0);
        }

        /**
         * @brief Request information about the name of the robot.
         *
         * @param robotName where to write the data to
         * @return void
         */
        bool requestRobotName(RobotName *robotName) {
            return requestTelemetry(ROBOT_NAME, robotName, 0);
        }

        /**
         * @brief Request information about the video streams of the robot.
         * 
         * @param streams where to write the data to
         */
        bool requestVideoStreams(VideoStreams *streams) {
            return requestTelemetry(VIDEO_STREAMS, streams, 0);
        }

        /**
         * @brief Request information about the cameras of the robot.
         * 
         * @param camerainformation 
         */
        bool requestCameraInformation(CameraInformation *camerainformation) {
            return requestTelemetry(CAMERA_INFORMATION, camerainformation, 0);
        }

        bool requestMap(Map *map, const ChannelId &channel = 0, const float &overrideMaxLatency = 120);

        // bool requestMap(std::string *map, const ChannelId &channel = 0, const float &overrideMaxLatency = 120) {
        //     return requestBinary(MAP, map, TELEMETRY_REQUEST, channel, overrideMaxLatency);
        // }

        /**
         * @brief Request which movement commands (in which frames) are supported by the robot
         * 
         * @param frames 
         */
        bool requestControllableFrames(ControllableFrames *frames) {
            return requestTelemetry(CONTROLLABLE_FRAMES, frames, 0);
        }

        /**
         * @brief Request which files can be downloaded from the robot
         * 
         * @param files file definition for the result
         */
        bool requestAvailableFiles(FileDefinition *files) {
            return requestTelemetry(FILE_DEFINITION, files, 0);
        }

        /**
         * @brief download one of the defined files od folders
         * 
         * @param identifier identifier string from FileDefinition received by requestAvailableFiles()
         * @param compressed if true (and compiled with gzip) compredd filed before sending
         * @param targetpath local path where to save files (path from robot is preserved)
         */
        bool requestFile(const std::string &identifier, const bool &compressed = false, const std::string targetpath = "./", const float &overrideMaxLatency = 0);


        /**
         * @brief requestt a model file/definition from the robot, most probably an urdf
         * 
         * @param targetfolder folder to save the model information to
         * @return std::pair<std::string, std::string> robot model path and file to open
         */
        std::pair<std::string, std::string> requestRobotModel(const std::string &targetfolder = "./", const float &overrideMaxLatency = 120);


        /**
         * @brief Get the Number of pending messages for a specific Telemetry type
         * 
         * @param TelemetryMessageType  
         * @return unsigned int number of messages in the buffer
         */
        unsigned int getBufferSize(const TelemetryMessageType &type, const ChannelId &channel = 0) {
            int size = buffers->lockedAccess().get()[type][channel]->size();
            return size;
        }

        /**
         * @brief Get the TelemetryMessages
         * @warning This should not be called directly
         * 
         * @tparam DATATYPE 
         * @param type 
         * @param data 
         * @return unsigned int 
         */

        template< class DATATYPE > unsigned int getTelemetry(const MessageId &type, DATATYPE *data, bool onlyNewest, const ChannelId &channel) {
            auto lockedbuffer = buffers->lockedAccess();
            if (channel > 0 && channel > lockedbuffer.get()[type].size()-1) {
                // channel nonexistent, retrun 0, as the buffer might be created later on the first message
                return 0;
            }
            return RingBufferAccess::popData(lockedbuffer.get()[type][channel], data, onlyNewest);
        }

        unsigned int getTelemetryRaw(const MessageId &type, std::string *dataSerialized, bool onlyNewest, const ChannelId &channel) {
            if (channel > 0 && channel > buffers->lockedAccess().get()[type].size()-1) {
                // channel nonexistent, retrun 0, as the buffer might be created later on the first message
                return 0;
            }
            *dataSerialized = buffers->peekSerialized(type, channel, serializationMode);
            bool result = buffers->lockedAccess().get()[type][channel]->pop(onlyNewest);
            return result;
        }

        template< class DATATYPE > bool requestTelemetry(const TelemetryMessageType &type, DATATYPE *result, const ChannelId &channel) {
            std::string replybuf;
            bool received = requestBinary(type, &replybuf, TELEMETRY_REQUEST, channel);

            if (serializationMode == JSON) {
                google::protobuf::util::JsonStringToMessage(replybuf, result);
            }else{
                result->ParseFromString(replybuf);
            }
            return received;
        }

        template< class DATATYPE > void addTelemetryReceivedCallback(const MessageId &type, const std::function<void(const DATATYPE & data)> &function, const ChannelId &channel = 0) {
            RingBufferAccess::addDataReceivedCallback<DATATYPE>(buffers->lockedAccess().get()[type][channel], function);
        }

        void addTelemetryReceivedCallback(const std::function<void(const MessageId &type)> &function) {
            telemetryReceivedCallbacks.push_back(function);
        }


        bool requestBinary(const TelemetryMessageType &type, std::string *result, const ControlMessageType &requestType = TELEMETRY_REQUEST, const ChannelId &channel = 0, const float &overrideMaxLatency = 0);
        bool requestBinary(const std::string &request, std::string *result, const ControlMessageType &requestType = TELEMETRY_REQUEST, const float &overrideMaxLatency = 0);


        template <class PROTOREQ, class PROTOREP> bool requestProtobuf(const PROTOREQ& requestdata, PROTOREP *reply, const ControlMessageType &requestType, const float &overrideMaxLatency = 0) {
            std::string request, recvbuf;
            
            if (serializationMode == JSON) {
                google::protobuf::util::JsonPrintOptions jsonOptions;
                // jsonOptions.add_whitespace = true;
                jsonOptions.always_print_primitive_fields = true;
                google::protobuf::util::MessageToJsonString(requestdata, &request, jsonOptions); 
            }else{
                requestdata.SerializeToString(&request);
            }

            requestBinary(request, &recvbuf, requestType, overrideMaxLatency);

            if (serializationMode == JSON) {
                google::protobuf::util::JsonStringToMessage(recvbuf, reply);
            }else{
                google::protobuf::io::CodedInputStream cistream(reinterpret_cast<const uint8_t *>(recvbuf.data()), recvbuf.size());
                cistream.SetTotalBytesLimit(recvbuf.size());
                reply->ParseFromCodedStream(&cistream);    
            }

            return (recvbuf.size() > 0) ? true : false;
        }

       /**
         * @brief Get the Statistics object, it is only updated with data if this library is compiled with RRC_STATISTICS
         * 
         * @return Statistics& 
         */
        Statistics& getStatistics() {
            return statistics;
        }



    protected:
        virtual std::string sendRequest(const std::string& serializedMessage, const float &overrideMaxLatency = 0, const robot_remote_control::Transport::Flags &flags = robot_remote_control::Transport::NOBLOCK);

        TelemetryMessageType evaluateTelemetry(const std::string& reply);

        void updateStatistics(const uint32_t &bytesSent, const MessageId &type);

        TransportSharedPtr commandTransport;
        TransportSharedPtr telemetryTransport;

        float heartBeatDuration;
        Timer heartBeatTimer;
        std::atomic<float> heartBeatRoundTripTime;
        Timer latencyTimer;
        Timer requestTimer;
        Timer lastConnectedTimer;
        std::mutex commandTransportMutex;
        float maxLatency;

        std::shared_ptr<TelemetryBuffer>  buffers;
        std::function<void(const float&)> lostConnectionCallback;
        std::function<void()> connectedCallback;
        std::vector< std::function<void(const MessageId &type)> > telemetryReceivedCallbacks;
        std::atomic<bool> connected;

        Statistics statistics;

        std::array<ChannelId, TELEMETRY_MESSAGE_TYPES_NUMBER> messageChannels;

        std::array<std::vector<std::string>, TELEMETRY_MESSAGE_TYPES_NUMBER > messageChannelNames;
        std::array<std::map<std::string, ChannelId>, TELEMETRY_MESSAGE_TYPES_NUMBER > messageChannelIdByName;

        SerializationMode serializationMode;

        ControlMessage initControlMessage(const ControlMessageType &type, const std::string &data);

        template< class CLASS > ControlMessage initControlMessage(const ControlMessageType &type, const CLASS &protodata) {
            ControlMessage controlMessage;
            controlMessage.set_type(type);

            if (serializationMode == JSON) {
                google::protobuf::util::JsonPrintOptions jsonOptions;
                // jsonOptions.add_whitespace = true;
                jsonOptions.always_print_primitive_fields = true;
                google::protobuf::util::MessageToJsonString(protodata, controlMessage.mutable_json(), jsonOptions); 
            }else{
                protodata.SerializeToString(controlMessage.mutable_data());
            }
            return controlMessage;
        }

        template< class CLASS > std::string sendProtobufData(const CLASS &protodata, const ControlMessageType &type, const robot_remote_control::Transport::Flags &flags = robot_remote_control::Transport::NOBLOCK ) {
            std::string buf;
            ControlMessage controlMessage = initControlMessage(type, protodata);

            if (serializationMode == JSON) {
                google::protobuf::util::JsonPrintOptions jsonOptions;
                // jsonOptions.add_whitespace = true;
                jsonOptions.always_print_primitive_fields = true;
                google::protobuf::util::MessageToJsonString(controlMessage, &buf, jsonOptions); 
            }else{
                controlMessage.SerializeToString(&buf);
            }
            
            return sendRequest(buf, 0, flags);
        }

        class TelemetryAdderBase{
         public:
            explicit TelemetryAdderBase(std::shared_ptr<TelemetryBuffer> buffers) : overwrite(true), buffers(buffers) {}
            virtual ~TelemetryAdderBase() {}
            virtual void addToTelemetryBuffer(const MessageId &type, const std::string &serializedMessage, const ChannelId &channel, bool textMode) = 0;
            void setOverwrite(bool mode = true) {
                overwrite = mode;
            }

         protected:
            bool overwrite;
            std::shared_ptr<TelemetryBuffer>  buffers;
        };

        template <class CLASS> class TelemetryAdder : public TelemetryAdderBase {
         public:
            explicit TelemetryAdder(std::shared_ptr<TelemetryBuffer> buffers) : TelemetryAdderBase(buffers) {}
            virtual void addToTelemetryBuffer(const MessageId &type, const std::string &serializedMessage, const ChannelId &channel, bool textMode) {
                CLASS data;
                
                if (textMode) {
                    google::protobuf::util::JsonStringToMessage(serializedMessage, &data);
                }else{
                    data.ParseFromString(serializedMessage);
                }
                RingBufferAccess::pushData(buffers->lockedAccess().get()[type][channel], data, overwrite);
            }
        };

        std::vector< std::shared_ptr<TelemetryAdderBase> > telemetryAdders;

        template <class PROTO> void registerTelemetryType(const MessageId &type, const size_t &buffersize = 10) {
            buffers->registerType<PROTO>(type, buffersize);
            if (type >= telemetryAdders.size()) {  // e.g. type == 42, for index 42, size must be 43
                telemetryAdders.resize(type+1);
            }
            telemetryAdders[type] = std::shared_ptr<TelemetryAdderBase>(new TelemetryAdder<PROTO>(buffers));
            #ifdef RRC_STATISTICS
                PROTO telemetry_type;
                statistics.names[type] = telemetry_type.GetTypeName();
            #endif
        }

};

}  // end namespace robot_remote_control

