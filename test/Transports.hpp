#pragma once

#include "../src/Transports/ZeroMQ/TransportZmq.hpp"

#ifdef TRANSPORT_DEFAULT_GZIP
  #include "../src/TransportWrappers/Gzip/TransportWrapperGzip.hpp"
#endif
#ifdef TRANSPORT_UDT
  #include "../src/Transports/UDT/TransportUDT.hpp"
#endif
#ifdef TRANSPORT_WS
  #include "../src/Transports/WebSocket/TransportWebSocket.hpp"
#endif
#ifdef TRANSPORT_WS_JSON
  #include "../src/Transports/WebSocket/TransportWebSocket.hpp"
#endif
#ifdef TRANSPORT_WEB
  #include "../src/Transports/WebSocket/TransportWebSocket.hpp"
  #include "../src/Transports/Http/TransportHttp.hpp"
#endif
namespace robot_remote_control {

class Transports {
  public:
    // use as singleton
    static Transports& instance() {
      static Transports t;
      return t;
    }
  private:
    Transports();
  public:

    TransportSharedPtr robotControllerCommands;
    TransportSharedPtr robotControllerTelemetry;

    TransportSharedPtr controlledRobotCommands;
    TransportSharedPtr controlledRobotTelemetry;

};

}
