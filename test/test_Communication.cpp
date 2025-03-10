#include <boost/test/unit_test.hpp>

#include "../src/Transports/TransportZmq.hpp"

#ifdef TRANSPORT_DEFAULT_GZIP
  #include "../src/Transports/TransportWrapperGzip.hpp"
#endif
#ifdef TRANSPORT_UDT
  #include "../src/Transports/TransportUDT.hpp"
#endif

#include "TypeGenerator.hpp"

#include <iostream>
#include <atomic>

#define private public // :-|
#define protected public // :-|
#include "../src/RobotController/RobotController.hpp"
#include "../src/ControlledRobot/ControlledRobot.hpp"

#include "../src/Tools/TelemetryCallbackThread.hpp"

//use boost::filesystem instead of std::filesystem here (tests have boost dependency anyways)
#include <boost/filesystem.hpp>

using namespace robot_remote_control;

TransportSharedPtr commands;
TransportSharedPtr telemetry;

TransportSharedPtr command;
TransportSharedPtr telemetri;




#define COMPARE_PROTOBUF(VAR1, VAR2) BOOST_TEST(VAR1.SerializeAsString() == VAR2.SerializeAsString())

/**
 * @brief used to init the communication only when it is used, it is reused then.
 * 
 */
void initComms() {
  #ifdef TRANSPORT_DEFAULT
    if (!commands.get()) {
        printf("using zmq tcp\n");
        commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7003", TransportZmq::REQ));
    }
    if (!telemetry.get()) {telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7004", TransportZmq::SUB));}

    if (!command.get()) {command = TransportSharedPtr(new TransportZmq("tcp://*:7003", TransportZmq::REP));}
    if (!telemetri.get()) {telemetri = TransportSharedPtr(new TransportZmq("tcp://*:7004", TransportZmq::PUB));}
  #endif
  #ifdef TRANSPORT_DEFAULT_GZIP
    if (!commands.get()) {
        printf("using zmq tcp with gzip wrapper\n");
        commands = TransportSharedPtr(new TransportWrapperGzip(TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7003", TransportZmq::REQ))));
    }
    if (!telemetry.get()) {telemetry = TransportSharedPtr(new TransportWrapperGzip(TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7004", TransportZmq::SUB))));}

    if (!command.get()) {command = TransportSharedPtr(new TransportWrapperGzip(TransportSharedPtr(new TransportZmq("tcp://*:7003", TransportZmq::REP))));}
    if (!telemetri.get()) {telemetri = TransportSharedPtr(new TransportWrapperGzip(TransportSharedPtr(new TransportZmq("tcp://*:7004", TransportZmq::PUB))));}
  #endif
  #ifdef TRANSPORT_IPC
    if (!command.get()) {
        printf("using zmq IPC\n");
        command = TransportSharedPtr(new TransportZmq("ipc:///tmp/test0", TransportZmq::REP));
    }
    if (!telemetri.get()) {telemetri = TransportSharedPtr(new TransportZmq("ipc:///tmp/test1", TransportZmq::PUB));}
    if (!commands.get()) {commands = TransportSharedPtr(new TransportZmq("ipc:///tmp/test0", TransportZmq::REQ));}
    if (!telemetry.get()) {telemetry = TransportSharedPtr(new TransportZmq("ipc:///tmp/test1", TransportZmq::SUB));}
  #endif
  #ifdef TRANSPORT_UDT
    if (!command.get()) {
        printf("using UDT\n");
        command = TransportSharedPtr(new TransportUDT(TransportUDT::SERVER, 7001));
    }
    if (!telemetri.get()) {telemetri = TransportSharedPtr(new TransportUDT(TransportUDT::SERVER, 7002));}
    if (!commands.get()) {commands = TransportSharedPtr(new TransportUDT(TransportUDT::CLIENT, 7001, "127.0.0.1"));}
    if (!telemetry.get()) {
        telemetry = TransportSharedPtr(new TransportUDT(TransportUDT::CLIENT, 7002, "127.0.0.1"));
    }
  #endif
}

bool isFileEqual(const std::string& path1, const std::string& path2) {
    std::ifstream f1(path1, std::ios::in | std::ios::binary);
    std::ifstream f2(path2, std::ios::in | std::ios::binary);
    if (f1 && f2) {
        std::stringstream filestr1, filestr2;
        filestr1 << f1.rdbuf();
        filestr2 << f2.rdbuf();
        if (filestr1.str() != filestr2.str()) {
            std::cout << filestr1.str() << ":" << filestr2.str() << std::endl;
            return false;
        }
        return true;
    }
    return false;
}

template <class PROTOBUFDATA> PROTOBUFDATA testCommand(PROTOBUFDATA protodata, const ControlMessageType &type) {
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  robot.startUpdateThread(10);

  PROTOBUFDATA received;
  controller.sendProtobufData(protodata, type);

  // wait for command
  std::string recv;
  int count = 0;
  while (!robot.commandbuffers[type]->read(&recv)) {
    usleep(10000);
    if (count++ > 100) {
      BOOST_FAIL("did not receive data in time");
    }
  }


  robot.stopUpdateThread();
  received.ParseFromString(recv);

  // protodata.PrintDebugString();
  // std::cout << std::endl;
  // received.PrintDebugString();
  // std::cout << std::endl << std::endl << std::endl;

  return received;
}

BOOST_AUTO_TEST_CASE(check_pose_command) {
  // not using the set/get functions
  Pose posecmd, poserecv;
  posecmd = TypeGenerator::genPose();
  poserecv = testCommand(posecmd, TARGET_POSE_COMMAND);
  COMPARE_PROTOBUF(posecmd, poserecv);
}

BOOST_AUTO_TEST_CASE(check_twist_command) {
  Twist twistcmd, twistrecv;
  twistcmd = TypeGenerator::genTwist();
  twistrecv = testCommand(twistcmd, TWIST_COMMAND);
  COMPARE_PROTOBUF(twistcmd, twistrecv);
}

BOOST_AUTO_TEST_CASE(check_goto_command) {
  GoTo gotocmd, gotorecv;
  gotocmd = TypeGenerator::genGoTo();
  gotorecv = testCommand(gotocmd, GOTO_COMMAND);
  COMPARE_PROTOBUF(gotocmd, gotorecv);
}

BOOST_AUTO_TEST_CASE(check_joint_command) {
  JointState cmd, recv;
  cmd = TypeGenerator::genJointState();
  recv = testCommand(cmd, JOINTS_COMMAND);
  COMPARE_PROTOBUF(cmd, recv);
}

BOOST_AUTO_TEST_CASE(check_simple_action_command) {
  SimpleAction cmd, recv;
  cmd = TypeGenerator::genSimpleAction();
  recv = testCommand(cmd, SIMPLE_ACTIONS_COMMAND);
  COMPARE_PROTOBUF(cmd, recv);
}

BOOST_AUTO_TEST_CASE(check_complex_action_command) {
  ComplexAction cmd, recv;
  cmd = TypeGenerator::genComplexAction();
  recv = testCommand(cmd, COMPLEX_ACTION_COMMAND);
  COMPARE_PROTOBUF(cmd, recv);
}


BOOST_AUTO_TEST_CASE(checking_twist_command_transfer) {
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  robot.startUpdateThread(10);

  Twist sendtwistcommand, receivetwistcommand;
  sendtwistcommand.mutable_angular()->set_z(0.4);
  sendtwistcommand.mutable_linear()->set_x(0.6);

  controller.setTwistCommand(sendtwistcommand);

  // wait for command
  while (!robot.getTwistCommand(&receivetwistcommand)) {
    usleep(10000);
  }

  robot.stopUpdateThread();

  COMPARE_PROTOBUF(sendtwistcommand, receivetwistcommand);
}

BOOST_AUTO_TEST_CASE(checking_target_pose) {
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  Pose pose = TypeGenerator::genPose();
  Pose pose2;

  robot.startUpdateThread(10);

  controller.setTargetPose(pose);

  while (!robot.getTargetPoseCommand(&pose2)) {
    usleep(10000);
  }

  robot.stopUpdateThread();


  COMPARE_PROTOBUF(pose, pose2);
}

BOOST_AUTO_TEST_CASE(checking_current_pose) {
  initComms();
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);
  controller.update();
  usleep(100 * 1000);

  Pose pose = TypeGenerator::genPose();
  Pose currentpose;

  // buffer for size comparsion
  std::string buf;
  pose.SerializeToString(&buf);

  // send telemetry data
  int sent = robot.setCurrentPose(pose);

  // wait a little for data transfer
  // the Telemetry send is non-blocking in opposite to commands
  usleep(100 * 1000);
  while (!controller.getCurrentPose(&currentpose)) {
    // receive pending data
    controller.update();
    usleep(10000);
  }


  // data was sent completely
  BOOST_CHECK((unsigned int)sent == buf.size());
  // and is the same
  COMPARE_PROTOBUF(pose, currentpose);
}

BOOST_AUTO_TEST_CASE(checking_current_twist) {
  initComms();
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);
  controller.update();
  usleep(100 * 1000);

  Twist telemetry = TypeGenerator::genTwist();
  Twist currenttelemetry;

  // buffer for size comparsion
  std::string buf;
  telemetry.SerializeToString(&buf);

  // send telemetry data
  int sent = robot.setCurrentTwist(telemetry);

  // wait a little for data transfer
  // the Telemetry send is non-blocking in opposite to commands
  usleep(100 * 1000);
  // receive pending data
  controller.update();

  while (!controller.getCurrentTwist(&currenttelemetry)) {
    usleep(10000);
  }


  // data was sent completely
  BOOST_CHECK((unsigned int)sent == buf.size());
  // and is the same
  COMPARE_PROTOBUF(telemetry, currenttelemetry);
}

BOOST_AUTO_TEST_CASE(checking_current_acceleration) {
  initComms();
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);
  controller.update();
  usleep(100 * 1000);

  Acceleration telemetry = TypeGenerator::genAcceleration();
  Acceleration currenttelemetry;

  // buffer for size comparsion
  std::string buf;
  telemetry.SerializeToString(&buf);

  // send telemetry data
  int sent = robot.setCurrentAcceleration(telemetry);

  // wait a little for data transfer
  // the Telemetry send is non-blocking in opposite to commands
  usleep(100 * 1000);
  // receive pending data
  controller.update();

  while (!controller.getCurrentAcceleration(&currenttelemetry)) {
    usleep(10000);
  }


  // data was sent completely
  BOOST_CHECK((unsigned int)sent == buf.size());
  // and is the same
  COMPARE_PROTOBUF(telemetry, currenttelemetry);
}

BOOST_AUTO_TEST_CASE(generic_request_telemetry_data) {
  initComms();
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);
  controller.startUpdateThread(10);
  robot.startUpdateThread(10);

  Pose pose = TypeGenerator::genPose();
  JointState jointstate = TypeGenerator::genJointState();


  // send telemetry data
  robot.setCurrentPose(pose);
  robot.setJointState(jointstate);

  Pose telemetryPose;
  while (!controller.getCurrentPose(&telemetryPose)) {
    usleep(10000);
  }
  COMPARE_PROTOBUF(pose, telemetryPose);

  JointState telemetryJointstate;
  while (!controller.getCurrentJointState(&telemetryJointstate)) {
    usleep(10000);
  }
  COMPARE_PROTOBUF(jointstate, telemetryJointstate);


  // test single request
  Pose requestedpose;
  controller.requestTelemetry(CURRENT_POSE, &requestedpose, 0);
  COMPARE_PROTOBUF(pose, requestedpose);


  JointState requestedJointState;
  controller.requestTelemetry(JOINT_STATE, &requestedJointState, 0);
  COMPARE_PROTOBUF(jointstate, requestedJointState);


  robot.stopUpdateThread();
  controller.stopUpdateThread();
}

BOOST_AUTO_TEST_CASE(checking_log_message) {
  initComms();
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);
  controller.startUpdateThread(10);
  robot.startUpdateThread(10);

  LogMessage requested_log_message;



  // test debug message, should go through
  controller.setLogLevel(DEBUG);

  LogMessage debug_message;
  debug_message.set_level(DEBUG);
  debug_message.set_message("[DEBUG] This is a debug message.");
  robot.setLogMessage(debug_message);

  // wait a little for data transfer (Telemetry is non-blocking)
  while (!controller.getLogMessage(&requested_log_message)) {
    usleep(100 * 1000);
  }
  COMPARE_PROTOBUF(debug_message, requested_log_message);



  // test fatal message, should go through
  controller.setLogLevel(FATAL);

  LogMessage fatal_message;
  fatal_message.set_level(FATAL);
  fatal_message.set_message("[FATAL] This is a fatal message.");
  robot.setLogMessage(fatal_message);

  while (!controller.getLogMessage(&requested_log_message)) {
    usleep(100 * 1000);
  }
  COMPARE_PROTOBUF(fatal_message, requested_log_message);



  // test error message, should not go through (because LogLevel is still at FATAL)
  LogMessage error_message;
  error_message.set_level(ERROR);
  error_message.set_message("[ERROR] This is an error message.");
  robot.setLogMessage(error_message);

  // some time to be able to receive
  usleep(100 * 1000);
  bool received =  controller.getLogMessage(&requested_log_message);

  // no message should have arrived
  BOOST_CHECK_EQUAL(received, false);
  // compare if message is still the fatal message, not the error message
  COMPARE_PROTOBUF(fatal_message, requested_log_message);



  // test logLevel NONE, fatal message should not go through
  controller.setLogLevel(NONE);

  LogMessage another_fatal_message;
  another_fatal_message.set_level(FATAL);
  another_fatal_message.set_message("[FATAL] This is another fatal message.");
  robot.setLogMessage(another_fatal_message);

  // some time to be able to receive
  usleep(100 * 1000);
  received = controller.getLogMessage(&requested_log_message);

  // no message should have arrived
  BOOST_CHECK_EQUAL(received, false);
  // compare if message is still the first fatal message, not the second one
  COMPARE_PROTOBUF(fatal_message, requested_log_message);

  robot.stopUpdateThread();
  controller.stopUpdateThread();
}

BOOST_AUTO_TEST_CASE(checking_robot_state) {
  initComms();
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);
  controller.startUpdateThread(10);
  robot.startUpdateThread(10);

  std::vector<std::string> requested_robot_state;
  std::vector<std::string> received_robot_state;
  std::vector<std::string> second_requested_robot_state;



  // test basic getRobotState and requestRobotState
  robot.setRobotState("ROBOT_DEMO_RUNNING");

  while (!controller.getRobotState(&received_robot_state)) {
    usleep(10000);
  }
  controller.requestRobotState(&requested_robot_state);

  BOOST_TEST(requested_robot_state.front() == received_robot_state.front());


  bool result = controller.getRobotState(&received_robot_state);

  // should be empty because state was already recieved
  BOOST_CHECK_EQUAL(result, false);
  BOOST_TEST(received_robot_state.size() == 0);

  // test multiple requestRobotState calls
  robot.setRobotState("ROBOT_DEMO_FINISHED");
  usleep(100 * 1000);

  while (!controller.getRobotState(&received_robot_state)) {
    usleep(10000);
  }
  controller.requestRobotState(&requested_robot_state);
  controller.requestRobotState(&second_requested_robot_state);

  BOOST_TEST(received_robot_state.front() == "ROBOT_DEMO_FINISHED");
  BOOST_TEST(requested_robot_state.front() == received_robot_state.front());
  BOOST_TEST(requested_robot_state.front() == second_requested_robot_state.front());



  // test multiple setRobotState calls without getRobotState or requestRobotState inbetween
  robot.setRobotState("ROBOT_DEMO_RUNNING_AGAIN");
  usleep(100 * 1000);
  robot.setRobotState("ROBOT_DEMO_STOPPED");
  usleep(100 * 1000);

  controller.requestRobotState(&requested_robot_state);

  while (!controller.getRobotState(&received_robot_state)) {
    usleep(10000);
  }
  // request should return most recent state, get should return the oldest not retrieved one
  BOOST_TEST(requested_robot_state.front() == "ROBOT_DEMO_STOPPED");
  BOOST_TEST(received_robot_state.front() == "ROBOT_DEMO_RUNNING_AGAIN");


  while (!controller.getRobotState(&requested_robot_state)) {
    usleep(10000);
  }
  // now they should be equal
  BOOST_TEST(requested_robot_state.front() == requested_robot_state.front());


  robot.stopUpdateThread();
  controller.stopUpdateThread();
}


template <class PROTOBUFDATA> PROTOBUFDATA testTelemetry(PROTOBUFDATA protodata, const TelemetryMessageType &type) {
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(10);

  robot.sendTelemetry(protodata, type, false, 0);

  // wait for telemetry
  PROTOBUFDATA received;
  while (!controller.getTelemetry(type, &received, false, 0)) {
    usleep(10000);
  }

  // check if raw telemetry sending works
  std::string binarydata;
  PROTOBUFDATA receivedraw;
  protodata.SerializeToString(&binarydata);
  robot.sendTelemetryRaw(type, binarydata);
  while (!controller.getTelemetry(type, &receivedraw, false, 0)) {
    usleep(10000);
  }

  controller.stopUpdateThread();

  COMPARE_PROTOBUF(received, receivedraw);

  return received;
}

template <class PROTOBUFDATA> PROTOBUFDATA testRequest(PROTOBUFDATA protodata, const TelemetryMessageType &type, const ChannelId channel = 0)  {
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  robot.startUpdateThread(10);

  robot.sendTelemetry(protodata, type, false, channel);

  // check request
  PROTOBUFDATA received;
  controller.requestTelemetry(type, &received, channel);

  robot.stopUpdateThread();

  return received;
}


BOOST_AUTO_TEST_CASE(check_telemetry_pose) {
  // not using the set/get functions
  Pose send, recv;
  send = TypeGenerator::genPose();
  recv = testTelemetry(send, CURRENT_POSE);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_poses) {
  // not using the set/get functions
  Poses send, recv;
  send = TypeGenerator::genPoses();
  recv = testTelemetry(send, POSES);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_jointstate) {
  // not using the set/get functions
  JointState send, recv;
  send = TypeGenerator::genJointState();
  recv = testTelemetry(send, JOINT_STATE);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_controllablejoints) {
  // not using the set/get functions
  JointState send, recv;
  send = TypeGenerator::genJointState();
  recv = testTelemetry(send, CONTROLLABLE_JOINTS);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_simple_actions) {
  // not using the set/get functions
  SimpleActions send, recv;
  send = TypeGenerator::genSimpleActions();
  recv = testTelemetry(send, SIMPLE_ACTIONS);
  COMPARE_PROTOBUF(send, recv);
  recv = testRequest(send, SIMPLE_ACTIONS);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_complex_actions) {
  // not using the set/get functions
  ComplexActions send, recv;
  send = TypeGenerator::genComplexActions();
  recv = testTelemetry(send, COMPLEX_ACTIONS);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_request_robotname) {
  // not using the set/get functions
  RobotName send, recv;
  send.set_value("testname");
  recv = testRequest(send, ROBOT_NAME);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_request_videostreams) {
  // not using the set/get functions
  VideoStreams send, recv;
  send = TypeGenerator::genVideoStreams();
  recv = testRequest(send, VIDEO_STREAMS);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_wrechstate) {
  // not using the set/get functions
  WrenchState send, recv;
  send = TypeGenerator::genWrenchState();
  recv = testTelemetry(send, WRENCH_STATE);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_image) {
  // not using the set/get functions
  Image send, recv;
  send = TypeGenerator::genImage();
  recv = testTelemetry(send, IMAGE);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_imagelayers) {
  // not using the set/get functions
  ImageLayers send, recv;
  send = TypeGenerator::genImageLayers();
  recv = testTelemetry(send, IMAGE_LAYERS);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_request_camerainformation) {
  // not using the set/get functions
  CameraInformation send, recv;
  send = TypeGenerator::genCameraInformation();
  recv = testRequest(send, CAMERA_INFORMATION);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_odometry) {
  // not using the set/get functions
  Odometry send, recv;
  send = TypeGenerator::genOdometry();
  recv = testTelemetry(send, ODOMETRY);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_request_controllableframes) {
  // not using the set/get functions
  ControllableFrames send, recv;
  send = TypeGenerator::genControllableFrames();
  recv = testRequest(send, CONTROLLABLE_FRAMES);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_request_map) {

  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  GridMap grid;
  PointCloud cloud;

  Map map1, map2;

  ChannelId cloudchannel = robot.addChannel(MAP, "PointCloudMap");
  ChannelId gridchannel = robot.addChannel(MAP, "GridMap");

  robot.startUpdateThread(10);

  robot.setMap(cloud, cloudchannel);
  robot.setMap(grid, gridchannel);

  // check request

  // request channel
  robot_remote_control::ChannelsDefinition channels;
  controller.requestChannelsDefinition(&channels);

  BOOST_CHECK_EQUAL(channels.channel().Get(0).name(), "PointCloudMap");
  BOOST_CHECK_EQUAL(channels.channel().Get(0).channelno(), 1);
  BOOST_CHECK_EQUAL(channels.channel().Get(1).name(), "GridMap");
  BOOST_CHECK_EQUAL(channels.channel().Get(1).channelno(), 2);

  Map received_grid_map;
  Map received_cloud_map;

  controller.requestTelemetry(MAP, &received_cloud_map, 1);
  controller.requestTelemetry(MAP, &received_grid_map, 2);

  robot.stopUpdateThread();

  //unpack map
  GridMap received_grid;
  PointCloud received_cloud;

  received_cloud_map.map().UnpackTo(&received_cloud);
  received_grid_map.map().UnpackTo(&received_grid);

  COMPARE_PROTOBUF(grid, received_grid);
  COMPARE_PROTOBUF(cloud, received_cloud);
}


BOOST_AUTO_TEST_CASE(buffer_setting_overwrite) {
  initComms();

  // start with default buffer size and overwrite off (defaults)
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(0);
  robot.startUpdateThread(0);

  // use pose as telemetry dummy, x value is the counter
  Pose pose = TypeGenerator::genPose();
  Pose currentpose;

  controller.setSingleTelemetryBufferOverwrite(CURRENT_POSE, false);

  // fill buffer (overwrite is default off)
  for (int i = 0; i < 20; ++i) {
    pose.mutable_position()->set_x(i);
    robot.setCurrentPose(pose);
  }

  // wait a litte to let messages arrive
  while (controller.getDroppedTelemetry(CURRENT_POSE) < 10) {
    usleep(10000);
  }

  // read buffer (in buffersize)
  for (int i = 0; i < 10; ++i) {
    // buffer should be the first values
    if (controller.getCurrentPose(&currentpose)) {
      BOOST_TEST(i == currentpose.position().x());
    }
  }
  // buffer schould be empty
  BOOST_TEST(controller.getCurrentPose(&currentpose) == false);

  controller.setSingleTelemetryBufferOverwrite(CURRENT_POSE, true);

  // fill buffer (overwrite is default on)
  for (int i = 0; i < 25; ++i) {
    pose.mutable_position()->set_x(i);
    robot.setCurrentPose(pose);
  }
  // wait a litte to let messages arrive
  while (controller.getDroppedTelemetry(CURRENT_POSE) < 25) {
    usleep(100000);
  }
  // read buffer (in buffersize)
  for (int i = 15; i < 25; ++i) {
    if (controller.getCurrentPose(&currentpose)) {
      // buffer should be the last values
      BOOST_TEST(i == currentpose.position().x());
    }
  }
  // buffer schould be empty
  BOOST_TEST(controller.getCurrentPose(&currentpose) == false);
}

BOOST_AUTO_TEST_CASE(buffer_setting_resize) {
  initComms();

  // start with default buffer size and overwrite off (defaults)
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(0);
  robot.startUpdateThread(0);

  // use pose as telemetry dummy, x value is the counter
  Pose pose = TypeGenerator::genPose();
  Pose currentpose;

  // fill buffer (overwrite is default off)
  for (int i = 0; i < 20; ++i) {
    pose.mutable_position()->set_x(i);
    robot.setCurrentPose(pose);
  }

  // wait until all messages arrive (also the later dropped ones)
  while (controller.getDroppedTelemetry(CURRENT_POSE) < 10) {
    usleep(10000);
  }

  // resize buffer
  controller.setSingleTelemetryBufferSize(CURRENT_POSE, 5);
  // buffer schould be empty
  BOOST_TEST(controller.getCurrentPose(&currentpose) == false);

  // fill buffer (overwrite is default off)
  for (int i = 0; i < 10; ++i) {
    pose.mutable_position()->set_x(i);
    robot.setCurrentPose(pose);
  }

  // BOOST_TEST(controller.getDroppedTelemetry(CURRENT_POSE) == 15);

  // wait until messages arrives (buffer full)
  while (controller.getDroppedTelemetry(CURRENT_POSE) < 15) {
    usleep(10000);
  }


  // read buffer (in buffersize) (overwrite default on)
  for (int i = 5; i < 15; ++i) {
    // buffer should be the first values
    if (controller.getCurrentPose(&currentpose)) {
      BOOST_TEST(i == currentpose.position().x());
    }
  }

  // buffer schould be empty
  BOOST_TEST(controller.getCurrentPose(&currentpose) == false);

  controller.setSingleTelemetryBufferSize(CURRENT_POSE, 20);

  // fill buffer (overwrite is default off)
  for (int i = 0; i < 30; ++i) {
    pose.mutable_position()->set_x(i);
    robot.setCurrentPose(pose);
  }

  while (controller.getDroppedTelemetry(CURRENT_POSE) < 25) {
    usleep(10000);
  }

  // read buffer (in buffersize)
  for (int i = 10; i < 30; ++i) {
    // buffer should be the first values
    if (controller.getCurrentPose(&currentpose)) {
      BOOST_TEST_MESSAGE(std::to_string(currentpose.position().x()));
      BOOST_TEST(i == currentpose.position().x());
    }
  }

  // buffer schould be empty
  BOOST_TEST(controller.getCurrentPose(&currentpose) == false);
}

BOOST_AUTO_TEST_CASE(check_simple_sensors) {
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(10);
  robot.startUpdateThread(10);

  SimpleSensor temp_recv;

  // getting an unavaile sensor should return false
  bool result = controller.getSimpleSensor(&temp_recv);
  BOOST_TEST(result == false);


  // send unregistered sensor
  SimpleSensor temp;
  robot.setSimpleSensor(temp);


  while (!controller.getSimpleSensor(&temp_recv)) {
    usleep(10000);
  }
  COMPARE_PROTOBUF(temp, temp_recv);


  // init channels for sensors
  int c1 = robot.addChannel(robot_remote_control::SIMPLE_SENSOR, "temperature");  // channel 1
  int c2 = robot.addChannel(robot_remote_control::SIMPLE_SENSOR, "velocity");  // channel 2

  BOOST_CHECK_EQUAL(c1, 1);
  BOOST_CHECK_EQUAL(c2, 2);

  // send unregistered sensor
  SimpleSensor velocity, velocity_recv;
  velocity.add_value(std::rand());
  robot.setSimpleSensor(velocity, 2);

  while (!controller.getSimpleSensor(&velocity_recv, false, 2)) {
    usleep(10000);
  }
  COMPARE_PROTOBUF(velocity, velocity_recv);

  controller.stopUpdateThread();
}

BOOST_AUTO_TEST_CASE(check_callbacks) {
  Pose robotpose, controlpose;
  robotpose = TypeGenerator::genPose();
  controlpose = TypeGenerator::genPose();
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(10);
  robot.startUpdateThread(10);

  // add generic callback
  robot.addCommandReceivedCallback([](const MessageId &type){
      // not getting the pose here, isnew will fail for other callback
      BOOST_TEST(type == TARGET_POSE_COMMAND); //we are sending a pose below
  });

  // add command callback
  robot.addCommandReceivedCallback(TARGET_POSE_COMMAND, [controlpose, &robot](){
      Pose pose;
      bool isnew = robot.getTargetPoseCommand(&pose);
      COMPARE_PROTOBUF(controlpose, pose);
      BOOST_TEST(isnew == true);
  });
  // send the pose
  controller.setTargetPose(controlpose);

  controller.addTelemetryReceivedCallback<Pose>(CURRENT_POSE, [robotpose, &controller](const Pose & data){
      COMPARE_PROTOBUF(robotpose, data);
      Pose pose;
      bool isnew = controller.getCurrentPose(&pose);
      COMPARE_PROTOBUF(robotpose, pose);
      BOOST_TEST(isnew == true);
  });
  robot.setCurrentPose(robotpose);

}

BOOST_AUTO_TEST_CASE(check_callback_threads) {
  Pose robotpose, controlpose, robotpose2;
  robotpose = TypeGenerator::genPose();
  robotpose2 = TypeGenerator::genPose();
  controlpose = TypeGenerator::genPose();
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(0);
  robot.startUpdateThread(0);

  controller.setHeartBeatDuration(1);
  controller.waitForConnection();

  std::atomic<bool> wasBusy;
  wasBusy = false;
  std::atomic<bool> started;
  started = false;
  std::atomic<bool> wasRunning;
  wasRunning = false;

  // setup threaded callback
  auto dataCallback = [robotpose, &controller, &wasRunning, &started](const robot_remote_control::Pose &pose) {
      started = true;
      sleep(2); // simulate some calculation time (data needs to stay valid, so check afterwards)
      COMPARE_PROTOBUF(robotpose, pose);
      Pose currentpose;

      //in the 2 seconds of sleep, a new pose arrived, so it should be different when received via getCurrentPose() with onlynewest
      bool isnew = controller.getCurrentPose(&currentpose, true);
      BOOST_TEST(isnew == true);
      BOOST_TEST(robotpose.SerializeAsString() != currentpose.SerializeAsString());
      wasRunning = true;
  };

  auto busyCallabck = [&wasBusy]() {
    wasBusy = true;
  };

  robot_remote_control::TelemetryCallbackThread<robot_remote_control::Pose> threadedCallback(controller, robot_remote_control::CURRENT_POSE, dataCallback, 0, busyCallabck);

  // trigger initial callback (sleeps first)
  robot.setCurrentPose(robotpose);
  // after one second, it should definately be running
  while (!started) {
    usleep(100);
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
  }
  BOOST_TEST(threadedCallback.finished() == false); // callback should be running
  //BOOST_CHECK_EQUAL(wasBusy, false); // callback should not have been called a 2nd time

  // set a new pose, tries to trigger thread the 2nd time, now the newest pose in the buffer is different but the one given to the callback is the older one
  robot.setCurrentPose(robotpose2);
  
  // wait until 2nd call failed
  while (!wasBusy || threadedCallback.finished()) {
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    usleep(100);
  }
  BOOST_CHECK_EQUAL(wasBusy, true);

  //wait to let the initial callback finish
  while (!wasRunning) {
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
     usleep(100);
  }
  // wait some time after callback ended to let the implemnentation set the threadedCallback.finished() value
  sleep(1);
  
  BOOST_TEST(threadedCallback.finished() == true);

}



BOOST_AUTO_TEST_CASE(test_get_newest) {
  initComms();

  // start with default buffer size and overwrite off (defaults)
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(0);
  robot.startUpdateThread(0);

  // use pose as telemetry dummy, x value is the counter
  Pose pose = TypeGenerator::genPose();
  Pose currentpose;

  // fill buffer (overwrite is default off)
  for (int i = 0; i < 5; ++i) {
    pose.mutable_position()->set_x(i);
    robot.setCurrentPose(pose);
  }

  while (controller.getBufferSize(CURRENT_POSE) < 5) {
    usleep(10000);
  }
  usleep(100*1000);
  // gets the newest value
  // while (controller.getCurrentPose(&currentpose)){};
  controller.getCurrentPose(&currentpose, true);
  BOOST_CHECK_EQUAL(currentpose.position().x(), 4);

  // buffer schould be empty
  BOOST_CHECK_EQUAL(controller.getBufferSize(CURRENT_POSE), 0);
  BOOST_CHECK_EQUAL(controller.getCurrentPose(&currentpose, true), false);

  // test with buffer wrap (completely full)
  for (int i = 0; i < 10; ++i) {
    pose.mutable_position()->set_x(i);
    robot.setCurrentPose(pose);
  }
  while (controller.getBufferSize(CURRENT_POSE) < 10) {
    usleep(10000);
  }

  controller.getCurrentPose(&currentpose, true);
  BOOST_CHECK_EQUAL(currentpose.position().x(), 9);

  // buffer schould be empty
  BOOST_CHECK_EQUAL(controller.getBufferSize(CURRENT_POSE), 0);
  BOOST_CHECK_EQUAL(controller.getCurrentPose(&currentpose, true), false);


  // test with buffer wrap (overfull)
  for (int i = 0; i < 20; ++i) {
    pose.mutable_position()->set_x(i);
    robot.setCurrentPose(pose);
  }
  while (controller.getBufferSize(CURRENT_POSE) < 10) {
    usleep(10000);
  }
  // buffer full, but still receiving, wait more
  usleep(100 * 1000);

  controller.getCurrentPose(&currentpose, true);
  BOOST_CHECK_EQUAL(currentpose.position().x(), 19);

  // buffer schould be empty
  BOOST_CHECK_EQUAL(controller.getBufferSize(CURRENT_POSE), 0);
  BOOST_CHECK_EQUAL(controller.getCurrentPose(&currentpose, true), false);


}

BOOST_AUTO_TEST_CASE(file_transfer) {
    initComms();

    RobotController controller(commands, telemetry);
    ControlledRobot robot(command, telemetri);

    controller.startUpdateThread(0);
    robot.startUpdateThread(0);

    //should return false if no files set up
    BOOST_CHECK_EQUAL(controller.requestFile("folder", false, "./"), false);


    FileDefinition files;
    File* file;

    file = files.add_file();
    files.add_isfolder(true);
    file->set_identifier("folder");
    file->set_path("./test/testfiles/");

    file = files.add_file();
    files.add_isfolder(false);
    file->set_identifier("topfolderfile");
    file->set_path("./test/testfiles/topfolderfile");

    file = files.add_file();
    files.add_isfolder(false);
    file->set_identifier("subfolderfile");
    file->set_path("./test/testfiles/subfolder/subfolderfile");

    file = files.add_file();
    files.add_isfolder(false);
    file->set_identifier("renamedfile");
    file->set_path("./test/testfiles/subfolder/subfolderfile");
    file->set_remote_path("./renamed_file");

    file = files.add_file();
    files.add_isfolder(true);
    file->set_identifier("subfolder");
    file->set_path("./test/testfiles/subfolder");

    file = files.add_file();
    files.add_isfolder(true);
    file->set_identifier("renamedfolder");
    file->set_path("./test/testfiles/subfolder");
    file->set_remote_path("./renamedfolder");

    robot.initFiles(files);

    // set up how the received definitions should look like on the Controller side
    FileDefinition remote;
    remote.CopyFrom(files);
    for (auto& file : *remote.mutable_file()) {
        if (file.remote_path().size()) {
            // hide local path from RobotController
            file.set_path(file.remote_path());
            file.set_remote_path("");
        }
    }

    // Get Definition via Transport
    FileDefinition availablefiles;
    controller.requestAvailableFiles(&availablefiles);

    COMPARE_PROTOBUF(remote, availablefiles);

    // download folder
    BOOST_CHECK_EQUAL(controller.requestFile("folder", false, "./folder"), true);
    BOOST_TEST(boost::filesystem::exists("./folder/test/testfiles/topfolderfile"));
    BOOST_TEST(boost::filesystem::exists("./folder/test/testfiles/subfolder/subfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/topfolderfile", "./folder/test/testfiles/topfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/subfolder/subfolderfile", "./folder/test/testfiles/subfolder/subfolderfile"));

    // download single file
    BOOST_CHECK_EQUAL(controller.requestFile("topfolderfile", false, "./folder2"), true);
    BOOST_TEST(boost::filesystem::exists("./folder2/test/testfiles/topfolderfile"));
    BOOST_TEST(!boost::filesystem::exists("./folder2/test/testfiles/subfolder/subfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/topfolderfile", "./folder2/test/testfiles/topfolderfile"));


    // download single subfolder file
    BOOST_CHECK_EQUAL(controller.requestFile("subfolderfile", false, "./folder3"), true);
    BOOST_TEST(boost::filesystem::exists("./folder3/test/testfiles/subfolder/subfolderfile"));
    BOOST_TEST(!boost::filesystem::exists("./folder3/test/testfiles/topfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/subfolder/subfolderfile", "./folder3/test/testfiles/subfolder/subfolderfile"));

    // download renamed subfolder file (change path)
    BOOST_CHECK_EQUAL(controller.requestFile("renamedfile", false, "./folder4"), true);
    BOOST_TEST(boost::filesystem::exists("./folder4/renamed_file"));
    BOOST_TEST(isFileEqual("./test/testfiles/subfolder/subfolderfile", "./folder4/renamed_file"));

    // subfolder
    BOOST_CHECK_EQUAL(controller.requestFile("subfolder", false, "./folder5"), true);
    BOOST_TEST(boost::filesystem::exists("./folder5/test/testfiles/subfolder/subfolderfile"));

    // relocate folder
    BOOST_CHECK_EQUAL(controller.requestFile("renamedfolder", false, "./folder6"), true);
    BOOST_TEST(boost::filesystem::exists("./folder6/renamedfolder/subfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/subfolder/subfolderfile", "./folder6/renamedfolder/subfolderfile"));




    // cleanup
    boost::filesystem::remove_all("./folder");
    boost::filesystem::remove_all("./folder2");
    boost::filesystem::remove_all("./folder3");
    boost::filesystem::remove_all("./folder4");
    boost::filesystem::remove_all("./folder5");
    boost::filesystem::remove_all("./folder6");

    // with compression (test just the transfer, not renaming)
    // download folder
    BOOST_CHECK_EQUAL(controller.requestFile("folder", true, "./cfolder"), true);
    BOOST_TEST(boost::filesystem::exists("./cfolder/test/testfiles/topfolderfile"));
    BOOST_TEST(boost::filesystem::exists("./cfolder/test/testfiles/subfolder/subfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/topfolderfile", "./cfolder/test/testfiles/topfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/subfolder/subfolderfile", "./cfolder/test/testfiles/subfolder/subfolderfile"));

    // // download single file
    BOOST_CHECK_EQUAL(controller.requestFile("topfolderfile", true, "./cfolder2"), true);
    BOOST_TEST(boost::filesystem::exists("./cfolder2/test/testfiles/topfolderfile"));
    BOOST_TEST(!boost::filesystem::exists("./cfolder2/test/testfiles/subfolder/subfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/topfolderfile", "./cfolder2/test/testfiles/topfolderfile"));

    // download single subfolder file
    BOOST_CHECK_EQUAL(controller.requestFile("subfolderfile", true, "./cfolder3"), true);
    BOOST_TEST(boost::filesystem::exists("./cfolder3/test/testfiles/subfolder/subfolderfile"));
    BOOST_TEST(!boost::filesystem::exists("./cfolder3/test/testfiles/topfolderfile"));
    BOOST_TEST(isFileEqual("./test/testfiles/subfolder/subfolderfile", "./cfolder3/test/testfiles/subfolder/subfolderfile"));

    // cleanup
    boost::filesystem::remove_all("./cfolder");
    boost::filesystem::remove_all("./cfolder2");
    boost::filesystem::remove_all("./cfolder3");

    // non-existent ID leads to error
    BOOST_CHECK_EQUAL(controller.requestFile("", false, "./"), false);
    BOOST_CHECK_EQUAL(controller.requestFile("nonexist", false, "./"), false);




}

#ifdef TRANSPORT_DEFAULT
BOOST_AUTO_TEST_CASE(connection_loss_and_reconnect) {
    initComms();

    RobotController controller(commands, telemetry);
    ControlledRobot robot(command, telemetri);

    controller.startUpdateThread(0);
    robot.startUpdateThread(0);

    bool detected_on_controller = false;
    bool detected_on_robot = false;

    controller.setHeartBeatDuration(0.1);

    controller.setupDisconnectedCallback([&](const float& since) {
      detected_on_controller = true;
    });
    int controllerConnections = 0;
    controller.setupConnectedCallback([&](){
      controllerConnections++;
    });
    robot.setupDisconnectedCallback(0.1, [&](const float& since) {
      detected_on_robot = true;
    });
    int robotConnections = 0;
    robot.setupConnectedCallback([&](){
      robotConnections++;
    });


    // wait for connection
    while (!controller.isConnected()) {
      usleep(10000);
    }

    BOOST_CHECK_EQUAL(controller.isConnected(), true);
    BOOST_CHECK_EQUAL(robot.isConnected(), true);

    BOOST_CHECK_EQUAL(controllerConnections, 1);
    BOOST_CHECK_EQUAL(robotConnections, 1);

    auto zmqptr = std::dynamic_pointer_cast<TransportZmq>(commands);
    zmqptr->disconnect();

    usleep(300000);
    BOOST_TEST(detected_on_controller);
    BOOST_TEST(detected_on_robot);

    BOOST_CHECK_EQUAL(controller.isConnected(), false);
    BOOST_CHECK_EQUAL(robot.isConnected(), false);

    zmqptr->connect();
    // wait for connection
    while (!controller.isConnected()) {
      usleep(10000);
    }

    BOOST_CHECK_EQUAL(controller.isConnected(), true);
    BOOST_CHECK_EQUAL(robot.isConnected(), true);

    BOOST_CHECK_EQUAL(controllerConnections, 2);
    BOOST_CHECK_EQUAL(robotConnections, 2);
}
#endif


BOOST_AUTO_TEST_CASE(robot_model) {
    initComms();

    RobotController controller(commands, telemetry);
    ControlledRobot robot(command, telemetri);

    controller.startUpdateThread(0);
    robot.startUpdateThread(0);

    // should return false if no files set up
    BOOST_CHECK_EQUAL(controller.requestRobotModel("./downloaded_model_folder").first, "");
    BOOST_CHECK_EQUAL(controller.requestRobotModel("./downloaded_model_folder").second, "");


    FileDefinition model;
    File* file;

    file = model.add_file();
    model.add_isfolder(true);
    file->set_identifier("robotmodel");
    file->set_path("./test/testfiles");

    robot.initRobotModel(model, "model/model.urdf");

    auto modelpath = controller.requestRobotModel("./downloaded_model_folder");
    BOOST_CHECK_EQUAL(modelpath.first, "./test/testfiles");
    BOOST_CHECK_EQUAL(modelpath.second, "model/model.urdf");
    BOOST_TEST(boost::filesystem::exists("./downloaded_model_folder/test/testfiles/model/model.urdf"));
    BOOST_TEST(isFileEqual("./test/testfiles/model/model.urdf", "./downloaded_model_folder/test/testfiles/model/model.urdf"));

    // cleanup
    boost::filesystem::remove_all("./downloaded_model_folder");
}


BOOST_AUTO_TEST_CASE(telemetry_channels) {
    initComms();

    RobotController controller(commands, telemetry);
    ControlledRobot robot(command, telemetri);

    robot.startUpdateThread(0);

    uint8_t channelno = robot.addChannel(robot_remote_control::CURRENT_POSE, "ManipulatorPose");
    // controller.addChannelBuffer<robot_remote_control::Pose>(robot_remote_control::CURRENT_POSE);

    // after creating the channel, it should be requestable
    robot_remote_control::ChannelsDefinition channels;
    controller.requestChannelsDefinition(&channels);

    BOOST_CHECK_EQUAL(channels.channel().Get(0).channelno(), 1);
    BOOST_CHECK_EQUAL(channels.channel().Get(0).messagetype(), robot_remote_control::CURRENT_POSE);
    BOOST_CHECK_EQUAL(channels.channel().Get(0).name(), "ManipulatorPose");

    uint8_t channelno2 = robot.addChannel(robot_remote_control::CURRENT_POSE, "ManipulatorPose2");

    controller.requestChannelsDefinition(&channels);
    BOOST_CHECK_EQUAL(channels.channel().Get(0).channelno(), 1);
    BOOST_CHECK_EQUAL(channels.channel().Get(0).messagetype(), robot_remote_control::CURRENT_POSE);
    BOOST_CHECK_EQUAL(channels.channel().Get(0).name(), "ManipulatorPose");
    BOOST_CHECK_EQUAL(channels.channel().Get(1).channelno(), 2);
    BOOST_CHECK_EQUAL(channels.channel().Get(1).messagetype(), robot_remote_control::CURRENT_POSE);
    BOOST_CHECK_EQUAL(channels.channel().Get(1).name(), "ManipulatorPose2");

    robot_remote_control::Pose pos1 = TypeGenerator::genPose();
    robot_remote_control::Pose pos2 = TypeGenerator::genPose();
    robot_remote_control::Pose pos3 = TypeGenerator::genPose();

    robot_remote_control::Pose receivedPose;

    // send telemetry data
    robot.setCurrentPose(pos1);
    robot.setCurrentPose(pos2, channelno);
    // robot.setCurrentPose(pos3, 42);  // invalid channel

    // wait a little for data transfer
    // the Telemetry send is non-blocking in opposite to commands
    usleep(100 * 1000);
    while (!controller.getCurrentPose(&receivedPose)) {
      // receive pending data
      controller.update();
      usleep(10000);
    }
    // channel 0 should be empty now
    BOOST_CHECK_EQUAL(controller.getCurrentPose(&receivedPose), false);

    // but channel 1 not
    while (!controller.getCurrentPose(&receivedPose, false, channelno)) {
      // receive pending data
      controller.update();
      usleep(10000);
    }

    // now tere are two messages in two channles
    // try to request
    robot_remote_control::Pose rpos1 = TypeGenerator::genPose();
    robot_remote_control::Pose rpos2 = TypeGenerator::genPose();
    robot_remote_control::Pose req_rpos1;
    robot_remote_control::Pose req_rpos2;

    robot.setCurrentPose(rpos1);
    robot.setCurrentPose(rpos2, channelno);
    controller.requestTelemetry(robot_remote_control::CURRENT_POSE, &req_rpos1, 0);
    controller.requestTelemetry(robot_remote_control::CURRENT_POSE, &req_rpos2, 1);

    COMPARE_PROTOBUF(req_rpos1, rpos1);
    COMPARE_PROTOBUF(req_rpos2, rpos2);


    // receiving non-existing channel retruns false
    // get is not thowing, as a channel buffer can could requeted before data was received
    BOOST_CHECK_EQUAL(controller.getCurrentPose(&receivedPose, false, 42), false);

    // try to send via non-existing channel thows, as the setup on the robot side must be valid
    BOOST_CHECK_THROW(robot.setCurrentPose(pos1, 42), std::out_of_range);



    // robot.addChannel<robot_remote_control::JointState>(robot_remote_control::CURRENT_POSE, "Invalid type");

}
