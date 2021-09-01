#pragma once

#include "Transport.hpp"
#include <string>
#include <memory>

namespace robot_remote_control {

class TransportWrapperGzip : public Transport {
 public:
    explicit TransportWrapperGzip(std::shared_ptr<Transport> transport, const int &compressionlevel);
    virtual ~TransportWrapperGzip() {}

    /**
     * @brief send date
     * 
     * @param buf the buffer to send
     * @param Flags flags the flags
     * @return int number of bytes sent
     */
    virtual int send(const std::string& uncompressed, Flags flags = NONE);

    /**
     * @brief receive data
     * 
     * @param buf buffer to fill on receive
     * @param Flags flags the flags
     * @return int 0 if no data received, size of data otherwise
     */
    virtual int receive(std::string* uncompressed, Flags flags = NONE);


 private:
    std::shared_ptr<Transport> transport;
    int compressionlevel;
};

} // end namespace robot_remote_control-transport_udt

