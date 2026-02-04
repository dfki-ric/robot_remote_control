
#include "WireProtocol.hpp"
#include <atomic>
#include "../UpdateThread/Timer.hpp"

namespace robot_remote_control {

class WireProtocolReply {
 public:
    WireProtocolReply(TransportSharedPtr transport, const float &maxLatency = 1);
    virtual ~WireProtocolReply() {}

    int send(const std::string& data);

    int getReply(std::string * result);

    bool isConnected() {
        return connected;
    }

 private:
    std::atomic<bool> connected;
    float maxLatency;
    Timer requestTimer;

};

}  // robot_remote_control
