
#include "../Transports/Transport.hpp"

namespace robot_remote_control {

class WireProtocol {
 public:
    WireProtocol(TransportSharedPtr transport) : transport(transport) {}
    virtual ~WireProtocol() {}

    int send(const std::string& data, const float &overrideMaxLatency = 0, const robot_remote_control::Transport::Flags &flags = robot_remote_control::Transport::NOBLOCK) = 0;

    int getReply(std::string * result) = 0;

    bool isConnected() = 0;

 private:
    TransportSharedPtr transport;


};

}  // robot_remote_control
