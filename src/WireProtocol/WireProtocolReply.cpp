
#include "WireProtocol.hpp"

using namespace robot_remote_control;


    WireProtocolReply::WireProtocolReply(TransportSharedPtr transport, const float &maxLatency = 1) : WireProtocol(transport) {

    }

    int WireProtocolReply::send(const std::string& data, const float &overrideMaxLatency = 0, const robot_remote_control::Transport::Flags &flags = robot_remote_control::Transport::NOBLOCK) {
        float currentMaxLatency = maxLatency;
        if (overrideMaxLatency > 0) {
            currentMaxLatency = overrideMaxLatency;
        }

        try {
            commandTransport->send(serializedMessage, flags);
        } catch (const std::exception &error) {
            connected.store(false);
            lostConnectionCallback(lastConnectedTimer.getElapsedTime());
            return "";
        }
        std::string replystr;

        requestTimer.start(currentMaxLatency);

        try {
            while (commandTransport->receive(&replystr, flags) == 0 && !requestTimer.isExpired()) {
                // wait time depends on how long the transports recv blocks
                usleep(1000);
            }
        } catch (const std::exception &error) {
            connected.store(false);
            lostConnectionCallback(lastConnectedTimer.getElapsedTime());
            return "";
        }
        if (replystr.size() == 0 && requestTimer.isExpired()) {
            connected.store(false);
            lostConnectionCallback(lastConnectedTimer.getElapsedTime());
            return replystr;
        }
        lastConnectedTimer.start();


    }

    int WireProtocolReply::getReply(std::string * result) {

    }

