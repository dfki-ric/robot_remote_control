#include <boost/test/unit_test.hpp>

#include <robot_remote_control/Transports/TransportZmq.hpp>

#include "TypeGenerator.hpp"

#include <iostream>

#define private public // :-|
#define protected public // :-|
#include "../src/RobotController/ExtendedRobotController.hpp"
#include "../src/ControlledRobot/ExtendedControlledRobot.hpp"

using namespace robot_remote_control;
using namespace myrobot;

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
}

// testCommand template called by runTest template
template <class PROTOBUFDATA, typename MESSAGETYPE, class CONTROLLER, class ROBOT> \
                 PROTOBUFDATA testCommand(PROTOBUFDATA protodata, const MESSAGETYPE &type, CONTROLLER * controller, ROBOT * robot) {

  robot->startUpdateThread(10);

  PROTOBUFDATA received;
  controller->sendProtobufData(protodata, type);

  // wait for command
  std::string recv;
  while (!robot->commandbuffers[type]->read(&recv)) {
    usleep(10000);
  }

  robot->stopUpdateThread();
  received.ParseFromString(recv);

  // protodata.PrintDebugString();
  // std::cout << std::endl;
  // received.PrintDebugString();
  // std::cout << std::endl << std::endl << std::endl;

  return received;
}
// testTelemetry template called by runTest template
template <class PROTOBUFDATA, typename MESSAGETYPE, class CONTROLLER, class ROBOT> \
                 PROTOBUFDATA testTelemetry(PROTOBUFDATA protodata, const MESSAGETYPE &type, CONTROLLER * controller, ROBOT * robot) {
  controller->startUpdateThread(10);

  robot->sendTelemetry(protodata, type, false, 0);

  // wait for telemetry
  PROTOBUFDATA received;
  while (!controller->getTelemetry(type, &received, false, 0)) {
    usleep(10000);
  }

  controller->stopUpdateThread();

  return received;
}
   
// handle message type, init required controller/robot objects, call testTelemetry or testCommand
template <class PROTOBUFDATA, typename MESSAGETYPE> PROTOBUFDATA runTest(PROTOBUFDATA protodata, const MESSAGETYPE &type) {
  initComms();
  bool extended_b = (std::is_same<MESSAGETYPE, ExtendedControlMessageType>::value || std::is_same<MESSAGETYPE, ExtendedTelemetryMessageType>::value);
  bool command_b  = (std::is_same<MESSAGETYPE, ExtendedControlMessageType>::value || std::is_same<MESSAGETYPE, ControlMessageType>::value );
  if(extended_b){
    ExtendedRobotController controller(commands, telemetry);
    ExtendedControlledRobot robot(command, telemetri);
    if (command_b){return testCommand(protodata, type, &controller, &robot);}
    else {return testTelemetry(protodata, type, &controller, &robot);}

  } else {
    RobotController controller(commands, telemetry);
    ControlledRobot robot(command, telemetri);
    if (command_b){return testCommand(protodata, type, &controller, &robot);}
    else {return testTelemetry(protodata, type, &controller, &robot);}
  }
}

// Actual tests:
  
BOOST_AUTO_TEST_CASE(check_twist_command) {
  // not using the set/get functions
  Twist twistcmd, twistrecv;
  twistcmd = TypeGenerator::genTwist();
  twistrecv = runTest(twistcmd, TWIST_COMMAND);
  COMPARE_PROTOBUF(twistcmd, twistrecv);
}

BOOST_AUTO_TEST_CASE(check_new_control) {
  // not using the set/get functions
  NewControlMessage send, recv;
  send = TypeGenerator::genNewControlMessage();
  recv = runTest(send, NEW_CONTROL_MESSAGE);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_telemetry_robotname) {
  // not using the set/get functions
  RobotName send, recv;
  send = TypeGenerator::genRobotName();
  recv = runTest(send, ROBOT_NAME);
  COMPARE_PROTOBUF(send, recv);
}

BOOST_AUTO_TEST_CASE(check_new_telemetry) {
  // not using the set/get functions
  NewTelemetryMessage send, recv;
  send = TypeGenerator::genNewTelemetryMessage();
  recv = runTest(send, NEW_TELEMETRY_MESSAGE);
  COMPARE_PROTOBUF(send, recv);
}

