#pragma once

#include <string>
#include <memory>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>

#include "../TransportWrapper.hpp"
#include "../../RingBuffer.hpp"
#include "../../UpdateThread/LockableClass.hpp"


namespace robot_remote_control {

/**
 * @brief wrapper to compress data before sending, might be useful in low bandwidth scenarios
 * @warning it adds an extra 4 bytes to each measage as size field, so the effect may be negative
 */

class MultiTransport : public TransportWrapper {
 public:
    enum Mode{COMMANDS,TELEMETRY};
    /**
     * @brief Construct a new Transport Wrapper Gzip object
     * 
     * @param transport 
     * @param compressionlevel 0-9 (-1 use default), 0 == no compression, 9 == best compression
     */
    MultiTransport(const std::vector<std::shared_ptr<Transport>> &transports, const Mode& mode, const size_t & buffersize);
   //  MultiTransport(std::shared_ptr<Transport> mainTransport, std::shared_ptr<Transport> additionalTransports, const size_t & buffersize);
    virtual ~MultiTransport() {}

    /**
     * @brief send data
     * 
     * @param buf the buffer to send
     * @param Flags flags the flags
     * @return int number of bytes sent
     */
    virtual int send(const std::string& msg, Transport::Flags flags = Transport::NONE);

    /**
     * @brief receive data
     * 
     * @param buf buffer to fill on receive
     * @param Flags flags the flags
     * @return int 0 if no data received, size of data otherwise
     */
    virtual int receive(std::string* msg, Transport::Flags flags = Transport::NONE);

    virtual bool requiresTextProtocol() override {
        return true;
    };


    virtual bool supportsControlledRobotTelemetry() {
        return supports & ROBOTTELEMETRY;
    }

    virtual bool supportsControlledRobotCommands() {
        return supports & ROBOTCOMMANDS;
    }

    virtual bool supportsRobotControllerTelemetry() {
        return supports & CONTROLLERTELEMETRY;
    }
    
    virtual bool supportsRobotControllerCommands() {
        return supports & CONTOLLERCOMMANDS;
    }

    virtual uint8_t getTransportSupportBitmask(){
        return supports;
    }


 private:

    void startThread(std::shared_ptr<Transport> t);

    Mode mode;
    std::vector<std::shared_ptr<Transport>> transports;
    std::vector<std::shared_ptr<std::thread>> receiveThreads;
    std::vector<std::shared_ptr<std::atomic<bool>>> receiveThreadsRunVars;

    Transport::Flags receiveFlags;
    size_t threadindex;

    
    struct Request {
      std::string msg;
      std::shared_ptr<Transport> transport;
    };

    std::mutex requesttmutex;
    std::shared_ptr<Transport> replytransport;


    LockableClass<RingBuffer<Request>> pendingMessages;
};

} // end namespace robot_remote_control

