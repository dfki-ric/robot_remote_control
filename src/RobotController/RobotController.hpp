#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"
#include "TelemetryBuffer.hpp"
#include "SimpleBuffer.hpp"
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

        /**
         * @brief in case there is a telemetry connection, receive all and fill the data fields
         */
        virtual void update();

        /**
         * @brief Set the overwrite mode of the buffers (default false)
         * 
         * @param type TelemetryMessageType enum
         * @param overwrite if false: new data is dropped if the buffer is full, if true: oldest data in buffer is overwritten
         */
        bool setSingleTelemetryBufferOverwrite(TelemetryMessageType type, bool overwrite = true);

        /**
         * @brief Set the buffer size of a sinlgle telemetry type (default value set in RobotController() constructor)
         * 
         * @warning buffer will be emptied on resize
         * 
         * @param type TelemetryMessageType enum
         * @param newsize the new size of the buffer 
         */
        bool setSingleTelemetryBufferSize(TelemetryMessageType type, uint16_t newsize = 10);

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
            lostConnectionCallback = callback;
        }

        /**
         * @brief return then current state of the connection using the heartbeat.
         * @return the current status of the connection, if a hearbeat duration iss set using setHeartBeatDuration()
         * 
         */
        bool isConnected() {
            return connected.load();
        }

        /**
         * @brief Get the Heart Breat Round Trip Time
         * 
         * @return float time in seconds needed to send/receive the last heartbeat (if used)
         */
        float getHeartBreatRoundTripTime() {
            return heartBreatRoundTripTime.load();
        }

        /**
         * @brief Get the number of objects in the buffer
         * 
         * @param type 
         * @return uint32_t 
         */
        uint32_t getTelemetryBufferDataSize(const TelemetryMessageType &type);

        /**
         * @brief Get the messages dropped of a specific because of full buffer
         * 
         * @param type 
         * @return uint32_t 
         */
        size_t getDroppedTelemetry(const TelemetryMessageType &type);

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
        void setLogLevel(const uint16_t &level);

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
        bool getCurrentPose(Pose *pose, bool onlyNewest = false) {
            return getTelemetry(CURRENT_POSE, pose, onlyNewest);
        }

        /**
         * @brief Get the Current Twist object
         * 
         * @param telemetry the Twist object to write to
         * @return true if new data was read
         * @return false otherwise
         */
        bool getCurrentTwist(Twist *telemetry, bool onlyNewest = false) {
            return getTelemetry(CURRENT_TWIST, telemetry, onlyNewest);
        }

        /**
         * @brief Get the Current Acceleration object
         *
         * @param telemetry the Acceleration object to write to
         * @return true if new data was read
         * @return false otherwise
         */
        bool getCurrentAcceleration(Acceleration *telemetry, bool onlyNewest = false) {
            return getTelemetry(CURRENT_ACCELERATION, telemetry, onlyNewest);
        }

        /**
         * @brief Get an array of Poses
         * 
         * @param repeated field of poses to write the data to
         * @return bool true if new data was read
         */
        bool getPoses(Poses *poses, bool onlyNewest = false) {
            return getTelemetry(POSES, poses, onlyNewest);
        }

        /**
         * @brief Get the last sent joint state of the robot
         * 
         * @param jointState the JointState to write the data to
         * @return bool true if new data was read
         */
        bool getCurrentJointState(JointState *jointState, bool onlyNewest = false) {
            return getTelemetry(JOINT_STATE, jointState, onlyNewest);
        }

        /**
         * Get the last sent wrench state of the robot
         * 
         * @param wrenchState the WrenchState to write the data to
         * @return bool true if new data was read
         */
        bool getCurrentWrenchState(WrenchState* wrenchState, bool onlyNewest = false) {
            return getTelemetry(WRENCH_STATE, wrenchState, onlyNewest);
        }

        int getCurrentIMUState(IMU* imu, bool onlyNewest = false) {
            return getTelemetry(IMU_VALUES, imu, onlyNewest);
        }

        int getCurrentContactPoints(ContactPoints* points, bool onlyNewest = false) {
            return getTelemetry(CONTACT_POINTS, points, onlyNewest);
        }


        /**
         * @brief Get a Simple Sensor object
         * 
         * @param simplesensor SimpleSensor to write data to
         * @return true 
         * @return false 
         */
        bool getSimpleSensor(const uint16_t &id, SimpleSensor *simplesensor) {
            auto lockedAccess = simplesensorbuffer->lockedAccess();
            bool result = false;
            if (lockedAccess.get().size() > id) {
                std::shared_ptr<RingBufferBase> bufferptr = lockedAccess.get()[id];
                if (bufferptr.get()) {
                    result = RingBufferAccess::popData(bufferptr, simplesensor);
                }
            }
            return result;
        }

        bool getPermissionRequest(PermissionRequest* request) {
            return getTelemetry(PERMISSION_REQUEST, request);
        }

        /**
         * @brief Get the Log Message object
         * 
         * @param msg LogMessage to write the data to
         * @return bool true if new data was read
         */
        bool getLogMessage(LogMessage *msg) {
            return getTelemetry(LOG_MESSAGE, msg);
        }

        /**
         * @brief Get the Robot State
         * 
         * @param state the string to write the state to
         * @return bool true if new data was read
         */
        bool getRobotState(std::vector<std::string> *state) {
            RobotState protostate;
            int statesleft = getTelemetry(ROBOT_STATE, &protostate);
            state->clear();
            for (const std::string &line : protostate.state()) {
                state->push_back(line);
            }
            return statesleft;
        }

        bool getRobotState(RobotState *state) {
            return getTelemetry(ROBOT_STATE, state);
        }



        /**
         * @brief Get the current transforms
         *
         * @param Transforms object to write the transforms to
         * @return bool true if new data was read
         */
        bool getCurrentTransforms(Transforms *transforms, bool onlyNewest = false) {
            return getTelemetry(TRANSFORMS, transforms, onlyNewest);
        }

        /**
         * @brief Get the Point Cloud object
         * 
         * @param pointcloud 
         * @return true 
         * @return false 
         */
        bool getPointCloud(PointCloud *pointcloud, bool onlyNewest = false) {
            return getTelemetry(POINTCLOUD, pointcloud, onlyNewest);
        }

        bool getImage(Image *image, bool onlyNewest = false) {
            return getTelemetry(IMAGE, image, onlyNewest);
        }

        bool getImageLayers(ImageLayers *imagelayers, bool onlyNewest = false) {
            return getTelemetry(IMAGE_LAYERS, imagelayers, onlyNewest);
        }

        bool getOdometry(Odometry* telemetry, bool onlyNewest = false) {
            return getTelemetry(ODOMETRY, telemetry, onlyNewest);
        }

        /**
         * @brief request the curretn state instead of waiting for the first telemetry message
         * 
         * @param state the string to write the state to
         */
        bool requestRobotState(std::vector<std::string> *state) {
            RobotState protostate;
            bool result = requestTelemetry(ROBOT_STATE, &protostate);
            state->clear();
            for (const std::string &line : protostate.state()) {
                state->push_back(line);
            }
            return result;
        }

        /**
         * @brief Request information about the complex actions of the robot.
         * 
         * @param complexActions where to write the data to
         * @return void
         */
        bool requestComplexActions(ComplexActions *complexActions) {
            return requestTelemetry(COMPLEX_ACTIONS, complexActions);
        }

        /**
         * @brief Request information about the simple actions of the robot.
         * 
         * @param simpleActions where to write the data to
         * @return void
         */
        bool requestSimpleActions(SimpleActions *simpleActions) {
            return requestTelemetry(SIMPLE_ACTIONS, simpleActions);
        }

        /**
         * @brief Request information about the controllable joints of the robot.
         * 
         * @param jointState where to write the data to
         * @return void
         */
        bool requestControllableJoints(JointState *jointState) {
            return requestTelemetry(CONTROLLABLE_JOINTS, jointState);
        }

        /**
         * @brief Request information about the simple sensors of the robot.
         * 
         * @param sensors sensord array to wtite the information to
         */
        bool requestSimpleSensors(SimpleSensors *sensors) {
            return requestTelemetry(SIMPLE_SENSOR_DEFINITION, sensors);
        }

        /**
         * @brief Request information about the name of the robot.
         *
         * @param robotName where to write the data to
         * @return void
         */
        bool requestRobotName(RobotName *robotName) {
            return requestTelemetry(ROBOT_NAME, robotName);
        }

        /**
         * @brief Request information about the video streams of the robot.
         * 
         * @param streams where to write the data to
         */
        bool requestVideoStreams(VideoStreams *streams) {
            return requestTelemetry(VIDEO_STREAMS, streams);
        }

        /**
         * @brief Request information about the cameras of the robot.
         * 
         * @param camerainformation 
         */
        bool requestCameraInformation(CameraInformation *camerainformation) {
            return requestTelemetry(CAMERA_INFORMATION, camerainformation);
        }

        /**
         * @brief Request the current pose of the robot.
         *
         * @param Pose where to write the data to
         * @return void
         */
        bool requestCurrentPose(Pose *pose) {
            return requestTelemetry(CURRENT_POSE, pose);
        }

        bool requestMap(Map *map, const uint16_t &mapId);

        bool requestMap(std::string *map, const uint16_t &mapId) {
            return requestBinary(mapId, map, MAP_REQUEST);
        }

        /**
         * @brief Request which movement commands (in which frames) are supported by the robot
         * 
         * @param frames 
         */
        bool requestControllableFrames(ControllableFrames *frames) {
            return requestTelemetry(CONTROLLABLE_FRAMES, frames);
        }

        /**
         * @brief Get the Number of pending messages for a specific Telemetry type
         * 
         * @param TelemetryMessageType  
         * @return unsigned int number of messages in the buffer
         */
        unsigned int getBufferSize(const TelemetryMessageType &type) {
            int size = buffers->lockedAccess().get()[type]->size();
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

        template< class DATATYPE > unsigned int getTelemetry(const uint16_t &type, DATATYPE *data, bool onlyNewest = false) {
            bool result = RingBufferAccess::popData(buffers->lockedAccess().get()[type], data, onlyNewest);
            return result;
        }

        template< class DATATYPE > unsigned int getTelemetryRaw(const uint16_t &type, DATATYPE *data, std::string *dataSerialized, bool onlyNewest = false) {
            *dataSerialized = buffers->peekSerialized(type);
            bool result = RingBufferAccess::popData(buffers->lockedAccess().get()[type], data, onlyNewest);
            return result;
        }

        template< class DATATYPE > bool requestTelemetry(const uint16_t &type, DATATYPE *result, const uint16_t &requestType = TELEMETRY_REQUEST) {
            std::string replybuf;
            bool received = requestBinary(type, &replybuf, requestType);
            result->ParseFromString(replybuf);
            return received;
        }

        template< class DATATYPE > void addTelemetryReceivedCallback(const uint16_t &type, const std::function<void(const DATATYPE & data)> &function) {
            RingBufferAccess::addDataReceivedCallback<DATATYPE>(buffers->lockedAccess().get()[type], function);
        }

        void addTelemetryReceivedCallback(const std::function<void(const uint16_t &type)> &function) {
            telemetryReceivedCallbacks.push_back(function);
        }

        bool requestBinary(const uint16_t &type, std::string *result, const uint16_t &requestType = TELEMETRY_REQUEST) {
            std::string buf;
            buf.resize(sizeof(uint16_t)*2);

            uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf.data()));
            *data = requestType;
            data++;

            // add the requested type
            *data = type;
            *result = sendRequest(buf);
            return (result->size() > 0) ? true : false;
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
        virtual std::string sendRequest(const std::string& serializedMessage, const robot_remote_control::Transport::Flags &flags = robot_remote_control::Transport::NOBLOCK);

        TelemetryMessageType evaluateTelemetry(const std::string& reply);

        void updateStatistics(const uint32_t &bytesSent, const uint16_t &type);

        TransportSharedPtr commandTransport;
        TransportSharedPtr telemetryTransport;

        float heartBeatDuration;
        Timer heartBeatTimer;
        std::atomic<float> heartBreatRoundTripTime;
        Timer latencyTimer;
        Timer requestTimer;
        Timer lastConnectedTimer;
        std::mutex commandTransportMutex;
        float maxLatency;

        std::shared_ptr<TelemetryBuffer>  buffers;
        std::shared_ptr<SimpleBuffer <SimpleSensor> >  simplesensorbuffer;
        // void initBuffers(const unsigned int &defaultSize);

        std::function<void(const float&)> lostConnectionCallback;
        std::vector< std::function<void(const uint16_t &type)> > telemetryReceivedCallbacks;
        std::atomic<bool> connected;

        Statistics statistics;

        template< class CLASS > std::string sendProtobufData(const CLASS &protodata, const uint16_t &type, const robot_remote_control::Transport::Flags &flags = robot_remote_control::Transport::NOBLOCK ) {
            std::string buf;
            buf.resize(sizeof(uint16_t));
            uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf.data()));
            *data = type;
            protodata.AppendToString(&buf);
            return sendRequest(buf, flags);
        }


        class TelemetryAdderBase{
         public:
            explicit TelemetryAdderBase(std::shared_ptr<TelemetryBuffer> buffers) : buffers(buffers), overwrite(true) {}
            virtual ~TelemetryAdderBase() {}
            virtual void addToTelemetryBuffer(const uint16_t &type, const std::string &serializedMessage) = 0;
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
            virtual void addToTelemetryBuffer(const uint16_t &type, const std::string &serializedMessage) {
                CLASS data;
                data.ParseFromString(serializedMessage);
                RingBufferAccess::pushData(buffers->lockedAccess().get()[type], data, overwrite);
            }
        };

        std::vector< std::shared_ptr<TelemetryAdderBase> > telemetryAdders;

        void addToSimpleSensorBuffer(const std::string &serializedMessage);

        template <class PROTO> void registerTelemetryType(const uint16_t &type, const size_t &buffersize = 10) {
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

