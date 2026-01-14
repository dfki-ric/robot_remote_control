#pragma once

#include "../src/Transports/TransportZmq.hpp"

#ifdef TRANSPORT_DEFAULT_GZIP
  #include "../src/Transports/TransportWrapperGzip.hpp"
#endif
#ifdef TRANSPORT_UDT
  #include "../src/Transports/TransportUDT.hpp"
#endif
#ifdef TRANSPORT_WS
  #include "../src/Transports/TransportWebSocket.hpp"
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
