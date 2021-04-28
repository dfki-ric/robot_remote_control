#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "MessageTypes.hpp"
#include "Transports/Transport.hpp"
#include "TelemetryBuffer.hpp"
#include "SimpleBuffer.hpp"
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
         * @brief sets the expected next heartbeat time on the robot side
         * The value is trasmitted with the heartbeat message and is evaluated on the robot side, (stable) latency
         * it not an issue
         */
        void setHeartBeatDuration(const float &duration_seconds) {
            heartBeatDuration = duration_seconds;
            heartBeatTimer.start(heartBeatDuration);
        }

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
         * @brief Get the Heart Breat Round Trip Time
         * 
         * @return float time in seconds needed to send/receive the last heartbeat (if used)
         */
        float getHeartBreatRoundTripTime() {
            return heartBreatRoundTripTime.lockedAccess().get();
        }

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
        void setJointCommand(const JointState &jointsCommand);

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
        bool getCurrentPose(Pose *pose) {
            return getTelemetry(CURRENT_POSE, pose);
        }

        /**
         * @brief Get an array of Poses
         * 
         * @param repeated field of poses to write the data to
         * @return bool true if new data was read
         */
        bool getPoses(Poses *poses) {
            return getTelemetry(POSES, poses);
        }

        /**
         * @brief Get the last sent joint state of the robot
         * 
         * @param jointState the JointState to write the data to
         * @return bool true if new data was read
         */
        bool getCurrentJointState(JointState *jointState) {
            return getTelemetry(JOINT_STATE, jointState);
        }

        /**
         * Get the last sent wrench state of the robot
         * 
         * @param wrenchState the WrenchState to write the data to
         * @return bool true if new data was read
         */
        bool getCurrentWrenchState(WrenchState* wrenchState) {
            return getTelemetry(WRENCH_STATE, wrenchState);
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
        bool getRobotState(std::string *state) {
            RobotState protostate;
            int statesleft = getTelemetry(ROBOT_STATE, &protostate);
            *state = protostate.state();
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
        bool getCurrentTransforms(Transforms *transforms) {
            return getTelemetry(TRANSFORMS, transforms);
        }

        /**
         * @brief request the curretn state instead of waiting for the first telemetry message
         * 
         * @param state the string to write the state to
         */
        void requestRobotState(std::string *state) {
            RobotState protostate;
            requestTelemetry(ROBOT_STATE, &protostate);
            *state = protostate.state();
        }

        /**
         * @brief Request information about the complex actions of the robot.
         * 
         * @param complexActions where to write the data to
         * @return void
         */
        void requestComplexActions(ComplexActions *complexActions) {
            requestTelemetry(COMPLEX_ACTIONS, complexActions);
        }

        /**
         * @brief Request information about the simple actions of the robot.
         * 
         * @param simpleActions where to write the data to
         * @return void
         */
        void requestSimpleActions(SimpleActions *simpleActions) {
            requestTelemetry(SIMPLE_ACTIONS, simpleActions);
        }

        /**
         * @brief Request information about the controllable joints of the robot.
         * 
         * @param jointState where to write the data to
         * @return void
         */
        void requestControllableJoints(JointState *jointState) {
            requestTelemetry(CONTROLLABLE_JOINTS, jointState);
        }

        /**
         * @brief Request information about the simple sensors of the robot.
         * 
         * @param sensors sensord array to wtite the information to
         */
        void requestSimpleSensors(SimpleSensors *sensors) {
            requestTelemetry(SIMPLE_SENSOR_DEFINITION, sensors);
        }

        /**
         * @brief Request information about the name of the robot.
         *
         * @param robotName where to write the data to
         * @return void
         */
        void requestRobotName(RobotName *robotName) {
            requestTelemetry(ROBOT_NAME, robotName);
        }

        /**
         * @brief Request information about the video streams of the robot.
         * 
         * @param streams where to write the data to
         */
        void requestVideoStreams(VideoStreams *streams) {
            requestTelemetry(VIDEO_STREAMS, streams);
        }

        /**
         * @brief Request the current pose of the robot.
         *
         * @param Pose where to write the data to
         * @return void
         */
        void requestCurrentPose(Pose *pose) {
            requestTelemetry(CURRENT_POSE, pose);
        }

        void requestMap(Map *map, const uint16_t &mapId);

        void requestMap(std::string *map, const uint16_t &mapId){
            requestBinary(mapId, map, MAP_REQUEST);
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

        template< class DATATYPE > unsigned int getTelemetry(const uint16_t &type, DATATYPE *data ) {
            bool result = RingBufferAccess::popData(buffers->lockedAccess().get()[type], data);
            return result;
        }

        template< class DATATYPE > void requestTelemetry(const uint16_t &type, DATATYPE *result, const uint16_t &requestType = TELEMETRY_REQUEST) {
            std::string replybuf;
            requestBinary(type, &replybuf, requestType);
            result->ParseFromString(replybuf);

        }

        void requestBinary(const uint16_t &type, std::string *result, const uint16_t &requestType = TELEMETRY_REQUEST) {
            std::string buf;
            buf.resize(sizeof(uint16_t)*2);

            uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf.data()));
            *data = requestType;
            data++;

            // add the requested type
            *data = type;
            *result = sendRequest(buf);
        }


    protected:

        virtual std::string sendRequest(const std::string& serializedMessage, const robot_remote_control::Transport::Flags &flags = robot_remote_control::Transport::NOBLOCK);

        TelemetryMessageType evaluateTelemetry(const std::string& reply);

        TransportSharedPtr commandTransport;
        TransportSharedPtr telemetryTransport;

        float heartBeatDuration;
        Timer heartBeatTimer;
        ThreadProtectedVar<float> heartBreatRoundTripTime;
        Timer latencyTimer;
        Timer requestTimer;
        Timer lastConnectedTimer;
        std::mutex commandTransportMutex;
        float maxLatency;

        std::shared_ptr<TelemetryBuffer>  buffers;
        std::shared_ptr<SimpleBuffer <SimpleSensor> >  simplesensorbuffer;
        // void initBuffers(const unsigned int &defaultSize);

        std::function<void(const float&)> lostConnectionCallback;

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
            explicit TelemetryAdderBase(std::shared_ptr<TelemetryBuffer> buffers) : buffers(buffers) {}
            virtual ~TelemetryAdderBase() {}
            virtual void addToTelemetryBuffer(const uint16_t &type, const std::string &serializedMessage) = 0;
         protected:
            std::shared_ptr<TelemetryBuffer>  buffers;
        };
        template <class CLASS> class TelemetryAdder : public TelemetryAdderBase {
         public:
            explicit TelemetryAdder(std::shared_ptr<TelemetryBuffer> buffers) : TelemetryAdderBase(buffers) {}
            virtual void addToTelemetryBuffer(const uint16_t &type, const std::string &serializedMessage) {
                CLASS data;
                data.ParseFromString(serializedMessage);
                RingBufferAccess::pushData(buffers->lockedAccess().get()[type], data);
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
        }

};

}  // end namespace robot_remote_control

