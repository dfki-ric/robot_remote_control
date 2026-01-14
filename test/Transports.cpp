#include "Transports.hpp"


using namespace robot_remote_control;

/**
 * @brief used to init the communication only when it is used, it is reused then.
 * 
 */
Transports::Transports() {

  #ifdef TRANSPORT_DEFAULT
    if (!robotControllerCommands.get()) {
        printf("using zmq tcp\n");
        robotControllerCommands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7003", TransportZmq::REQ));
    }
    if (!robotControllerTelemetry.get()) {robotControllerTelemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7004", TransportZmq::SUB));}
    if (!controlledRobotCommands.get()) {controlledRobotCommands = TransportSharedPtr(new TransportZmq("tcp://*:7003", TransportZmq::REP));}
    if (!controlledRobotTelemetry.get()) {controlledRobotTelemetry = TransportSharedPtr(new TransportZmq("tcp://*:7004", TransportZmq::PUB));}
  #endif
  #ifdef TRANSPORT_DEFAULT_GZIP
    if (!robotControllerCommands.get()) {
        printf("using zmq tcp with gzip wrapper\n");
        robotControllerCommands = TransportSharedPtr(new TransportWrapperGzip(TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7003", TransportZmq::REQ))));
    }
    if (!robotControllerTelemetry.get()) {robotControllerTelemetry = TransportSharedPtr(new TransportWrapperGzip(TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7004", TransportZmq::SUB))));}
    if (!controlledRobotCommands.get()) {controlledRobotCommands = TransportSharedPtr(new TransportWrapperGzip(TransportSharedPtr(new TransportZmq("tcp://*:7003", TransportZmq::REP))));}
    if (!controlledRobotTelemetry.get()) {controlledRobotTelemetry = TransportSharedPtr(new TransportWrapperGzip(TransportSharedPtr(new TransportZmq("tcp://*:7004", TransportZmq::PUB))));}
  #endif
  #ifdef TRANSPORT_IPC
    if (!controlledRobotCommands.get()) {controlledRobotCommands = TransportSharedPtr(new TransportZmq("ipc:///tmp/test0", TransportZmq::REP));printf("using zmq IPC\n");}
    if (!controlledRobotTelemetry.get()) {controlledRobotTelemetry = TransportSharedPtr(new TransportZmq("ipc:///tmp/test1", TransportZmq::PUB));}
    if (!robotControllerCommands.get()) {robotControllerCommands = TransportSharedPtr(new TransportZmq("ipc:///tmp/test0", TransportZmq::REQ));}
    if (!robotControllerTelemetry.get()) {robotControllerTelemetry = TransportSharedPtr(new TransportZmq("ipc:///tmp/test1", TransportZmq::SUB));}
  #endif
  #ifdef TRANSPORT_UDT
    if (!controlledRobotCommands.get()) {controlledRobotCommands = TransportSharedPtr(new TransportUDT(TransportUDT::SERVER, 7001));printf("using UDT\n");}
    if (!controlledRobotTelemetry.get()) {controlledRobotTelemetry = TransportSharedPtr(new TransportUDT(TransportUDT::SERVER, 7002));}
    if (!robotControllerCommands.get()) {robotControllerCommands = TransportSharedPtr(new TransportUDT(TransportUDT::CLIENT, 7001, "127.0.0.1"));}
    if (!robotControllerTelemetry.get()) {robotControllerTelemetry = TransportSharedPtr(new TransportUDT(TransportUDT::CLIENT, 7002, "127.0.0.1"));}
  #endif
  #ifdef TRANSPORT_WS
    if (!controlledRobotCommands.get()) {controlledRobotCommands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::SERVER, 7001));printf("using WS\n");}
    if (!controlledRobotTelemetry.get()) {controlledRobotTelemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::SERVER, 7002));}
    if (!robotControllerCommands.get()) {robotControllerCommands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT, 7001, "127.0.0.1"));}
    if (!robotControllerTelemetry.get()) {robotControllerTelemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT, 7002, "127.0.0.1"));}
  #endif
  #ifdef TRANSPORT_WS_JSON
    if (!controlledRobotCommands.get()) {controlledRobotCommands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::SERVER_TEXT, 7001));printf("using WS JSON\n");}
    if (!controlledRobotTelemetry.get()) {controlledRobotTelemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::SERVER_TEXT, 7002));}
    if (!robotControllerCommands.get()) {robotControllerCommands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT_TEXT, 7001, "127.0.0.1"));}
    if (!robotControllerTelemetry.get()) {robotControllerTelemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT_TEXT, 7002, "127.0.0.1"));}
  #endif

}