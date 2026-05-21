#include "MultiTransport.hpp"
#include <unistd.h>

// todo remove
#include <zmq.hpp>
#include <cxxabi.h>

namespace robot_remote_control {

MultiTransport::MultiTransport( const std::vector<std::shared_ptr<Transport>> &transports,
                                const Mode& mode,
                                const size_t & buffersize):
                            TransportWrapper(transports.front()),
                            mode(mode),
                            transports(transports),
                            threadindex(0) {
    
    if (mode == COMMANDS) {
        setTransportSupport(ROBOTCOMMANDS);
    }
    if (mode == TELEMETRY) {
        setTransportSupport(ROBOTTELEMETRY);
    }

    pendingMessages.lockedAccess()->resize(buffersize);
    
    // start recv threads
    for (const auto& t : transports){
        startThread(t);
    }
}

// MultiTransport::MultiTransport(std::shared_ptr<Transport> mainTransport, std::shared_ptr<Transport> additionalTransport, const size_t & buffersize) {
//     MultiTransport(mainTransport, {additionalTransport}, buffersize);
// }

int MultiTransport::send(const std::string& msg, Transport::Flags flags) {
    // just send on all

    if (mode == COMMANDS) {
        if (replytransport) {
            // reply sent unlick to handle next request
            requesttmutex.unlock();
            printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__,abi::__cxa_demangle(typeid(*replytransport.get()).name(),NULL,NULL,NULL));
            return replytransport->send(msg, flags);
        }
    }
    
    if (mode == TELEMETRY) {
        for (const auto& t : transports) {
            t->send(msg, flags);
        }
        return msg.size();
    }

    return 0;
}

int MultiTransport::receive(std::string* uncompressed, Transport::Flags flags) {

    //TODO: replies should only be forwarded to requesting transport

    // block 

    // receive in threads and have own deque
    
    //TODO used in telemetry mode at all?

    Request r;
    if (pendingMessages.lockedAccess()->popData(&r)) {
        if (mode == COMMANDS) {
            // block unitl previous reply was sent
            requesttmutex.lock();
            replytransport = r.transport;
        }
        *uncompressed = r.msg;
        return uncompressed->size();
    }
    return 0;
}

void MultiTransport::startThread(std::shared_ptr<Transport> t) {
    std::shared_ptr<std::atomic<bool>> running = std::make_shared<std::atomic<bool>>();
    *running = true;
    receiveThreadsRunVars.push_back(running);
    
    std::shared_ptr<std::thread> thread = std::make_shared<std::thread>([t,running,this]() {
        while (*running) {
            Request r;
            try{
                if (t->receive(&r.msg)) {
                    printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__,abi::__cxa_demangle(typeid(*t.get()).name(),NULL,NULL,NULL));
                    r.transport = t;
                    this->pendingMessages.lockedAccess()->pushData(r);
                    printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__,abi::__cxa_demangle(typeid(*t.get()).name(),NULL,NULL,NULL));
                }else{
                    usleep(10000);
                }
            } catch (const zmq::error_t &e) {
                //TODO: stop recv in Command mode if request is pending and remove zmq ref
            }
            
        }
    });
    receiveThreads.push_back(thread);
}

}  // namespace robot_remote_control

