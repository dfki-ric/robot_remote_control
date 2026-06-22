#pragma once 

#include "../Transports/Transport.hpp"

namespace robot_remote_control {

class TransportWrapper: public Transport {
 public: 
    
    enum Flags {NONE = 0x0, NOBLOCK = 0x1};

    TransportWrapper(std::shared_ptr<Transport> transport):Transport(),transport(transport){};
    virtual ~TransportWrapper(){};

    /**
     * @brief send data
     * 
     * @param buf the buffer to send
     * @param Flags flags the flags
     * @return int number of bytes sent
     */
    virtual int send(const std::string& buf, Transport::Flags flags = Transport::NONE) = 0;

    /**
     * @brief receive data
     * 
     * @param buf buffer to fill on receive
     * @param Flags flags the flags
     * @return int 0 if no data received, size of data otherwise
     */
    virtual int receive(std::string* buf, Transport::Flags flags = Transport::NONE) = 0;


    bool supportsControlledRobotTelemetry() override {
        return transport->supportsControlledRobotTelemetry();
    }

    bool supportsControlledRobotCommands() override {
        return transport->supportsControlledRobotCommands();
    }

    bool supportsRobotControllerTelemetry() override {
        return transport->supportsRobotControllerTelemetry();
    }
    
    bool supportsRobotControllerCommands() override {
        return transport->supportsRobotControllerCommands();
    }

    uint8_t getTransportSupportBitmask() override {
        return transport->getTransportSupportBitmask();
    }

 protected:
    std::shared_ptr<Transport> transport;

};

}