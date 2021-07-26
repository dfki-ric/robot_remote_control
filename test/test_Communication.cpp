#include <boost/test/unit_test.hpp>

#include "../src/Transports/TransportZmq.hpp"

#include "TypeGenerator.hpp"

#include <iostream>

#define private public // :-|
#define protected public // :-|
#include "../src/RobotController/RobotController.hpp"
#include "../src/ControlledRobot/ControlledRobot.hpp"

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
  if (!commands.get()) {commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7003", TransportZmq::REQ));}
  if (!telemetry.get()) {telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7004", TransportZmq::SUB));}

  if (!command.get()) {command = TransportSharedPtr(new TransportZmq("tcp://*:7003", TransportZmq::REP));}
  if (!telemetri.get()) {telemetri = TransportSharedPtr(new TransportZmq("tcp://*:7004", TransportZmq::PUB));}
  // if (!command.get()) {command = TransportSharedPtr(new TransportZmq("ipc:///tmp/test0", TransportZmq::REP));}
  // if (!telemetri.get()) {telemetri = TransportSharedPtr(new TransportZmq("ipc:///tmp/test1", TransportZmq::PUB));}
  // if (!commands.get()) {commands = TransportSharedPtr(new TransportZmq("ipc:///tmp/test0", TransportZmq::REQ));}
  // if (!telemetry.get()) {telemetry = TransportSharedPtr(new TransportZmq("ipc:///tmp/test1", TransportZmq::SUB));}

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
  while (!robot.commandbuffers[type]->read(&recv)) {
    usleep(10000);
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
  // receive pending data
  controller.update();

  while (!controller.getCurrentPose(&currentpose)) {
    usleep(10000);
  }


  // data was sent completely
  BOOST_CHECK((unsigned int)sent == buf.size());
  // and is the same
  COMPARE_PROTOBUF(pose, currentpose);
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
  controller.requestTelemetry(CURRENT_POSE, &requestedpose);
  COMPARE_PROTOBUF(pose, requestedpose);


  JointState requestedJointState;
  controller.requestTelemetry(JOINT_STATE, &requestedJointState);
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
  std::vector<std::string> gotten_robot_state;
  std::vector<std::string> second_requested_robot_state;



  // test basic getRobotState and requestRobotState
  robot.setRobotState("ROBOT_DEMO_RUNNING");

  while (!controller.getRobotState(&gotten_robot_state)) {
    usleep(10000);
  }
  controller.requestRobotState(&requested_robot_state);

  BOOST_TEST(requested_robot_state.front() == gotten_robot_state.front());


  bool result = controller.getRobotState(&gotten_robot_state);

  // should be empty because state was already recieved
  BOOST_CHECK_EQUAL(result, false);
  BOOST_TEST(gotten_robot_state.size() == 0);



  // test multiple requestRobotState calls
  robot.setRobotState("ROBOT_DEMO_FINISHED");
  usleep(100 * 1000);

  while (!controller.getRobotState(&gotten_robot_state)) {
    usleep(10000);
  }
  controller.requestRobotState(&requested_robot_state);
  controller.requestRobotState(&second_requested_robot_state);

  BOOST_TEST(gotten_robot_state.front() == "ROBOT_DEMO_FINISHED");
  BOOST_TEST(requested_robot_state.front() == gotten_robot_state.front());
  BOOST_TEST(requested_robot_state.front() == second_requested_robot_state.front());



  // test multiple setRobotState calls without getRobotState or requestRobotState inbetween
  robot.setRobotState("ROBOT_DEMO_RUNNING_AGAIN");
  usleep(100 * 1000);
  robot.setRobotState("ROBOT_DEMO_STOPPED");
  usleep(100 * 1000);

  controller.requestRobotState(&requested_robot_state);

  while (!controller.getRobotState(&gotten_robot_state)) {
    usleep(10000);
  }
  // request should return most recent state, get should return the oldest not retrieved one
  BOOST_TEST(requested_robot_state.front() == "ROBOT_DEMO_STOPPED");
  BOOST_TEST(gotten_robot_state.front() == "ROBOT_DEMO_RUNNING_AGAIN");


  while (!controller.getRobotState(&gotten_robot_state)) {
    usleep(10000);
  }
  // now they should be equal
  BOOST_TEST(gotten_robot_state.front() == requested_robot_state.front());


  robot.stopUpdateThread();
  controller.stopUpdateThread();
}


template <class PROTOBUFDATA> PROTOBUFDATA testTelemetry(PROTOBUFDATA protodata, const TelemetryMessageType &type) {
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(10);

  robot.sendTelemetry(protodata, type);

  // wait for telemetry
  PROTOBUFDATA received;
  while (!controller.getTelemetry(type, &received)) {
    usleep(10000);
  }

  controller.stopUpdateThread();

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
}

BOOST_AUTO_TEST_CASE(check_telemetry_complex_actions) {
  // not using the set/get functions
  ComplexActions send, recv;
  send = TypeGenerator::genComplexActions();
  recv = testTelemetry(send, COMPLEX_ACTIONS);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_robotname) {
  // not using the set/get functions
  RobotName send, recv;
  send = TypeGenerator::genRobotName();
  recv = testTelemetry(send, ROBOT_NAME);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_videostreams) {
  // not using the set/get functions
  VideoStreams send, recv;
  send = TypeGenerator::genVideoStreams();
  recv = testTelemetry(send, VIDEO_STREAMS);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_wrechstate) {
  // not using the set/get functions
  WrenchState send, recv;
  send = TypeGenerator::genWrenchState();
  recv = testTelemetry(send, WRENCH_STATE);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_simple_sensors) {
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  controller.startUpdateThread(10);
  robot.startUpdateThread(10);

  // send unregistered sensor
  SimpleSensor temp;
  temp.set_id(1);
  robot.setSimpleSensor(temp);


  SimpleSensor temp_recv;
  while (!controller.getSimpleSensor(1, &temp_recv)) {
    // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    usleep(10000);
    // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
  }
  COMPARE_PROTOBUF(temp, temp_recv);


  robot_remote_control::SimpleSensors sensors;
  robot_remote_control::SimpleSensor* sens;
  sens = sensors.add_sensors();
  sens->set_name("temperature");
  sens->set_id(1);
  sens->mutable_size()->set_x(1);  // only single value

  sens = sensors.add_sensors();
  sens->set_name("velocity");
  sens->set_id(2);
  sens->mutable_size()->set_x(3);  // 3 value vector

  robot.initSimpleSensors(sensors);

  // send unregistered sensor
  SimpleSensor velocity, velocity_recv;
  velocity.set_id(2);
  velocity.add_value(std::rand());
  robot.setSimpleSensor(velocity);

  while (!controller.getSimpleSensor(2, &velocity_recv)) {
    usleep(10000);
  }
  COMPARE_PROTOBUF(velocity, velocity_recv);

  controller.stopUpdateThread();


}

// BOOST_AUTO_TEST_CASE(check_permissions) {
//   // not using the set/get functions


//   WrenchState send, recv;
//   send = TypeGenerator::genWrenchState();
//   recv = testTelemetry(send, WRENCH_STATE);
//   COMPARE_PROTOBUF(send, recv);
// }
