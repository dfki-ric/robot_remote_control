#pragma once

#include "TransportWrapper.hpp"
#include <string>
#include <memory>

namespace robot_remote_control {

/**
 * @brief wrapper to compress data before sending, might be useful in low bandwidth scenarios
 * @warning it adds an extra 4 bytes to each measage as size field, so the effect may be negative
 */

class TransportWrapperGzip : public TransportWrapper {
 public:
    /**
     * @brief Construct a new Transport Wrapper Gzip object
     * 
     * @param transport 
     * @param compressionlevel 0-9 (-1 use default), 0 == no compression, 9 == best compression
     */
    explicit TransportWrapperGzip(std::shared_ptr<Transport> transport, const int &compressionlevel = -1);
    virtual ~TransportWrapperGzip() {}

    /**
     * @brief send data
     * 
     * @param buf the buffer to send
     * @param Flags flags the flags
     * @return int number of bytes sent
     */
    virtual int send(const std::string& uncompressed, Transport::Flags flags = Transport::NONE);

    /**
     * @brief receive data
     * 
     * @param buf buffer to fill on receive
     * @param Flags flags the flags
     * @return int 0 if no data received, size of data otherwise
     */
    virtual int receive(std::string* uncompressed, Transport::Flags flags = Transport::NONE);


 private:
    int compressionlevel;
};

} // end namespace robot_remote_control
