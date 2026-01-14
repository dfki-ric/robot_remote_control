#include "TransportWrapperGzip.hpp"
#include "../Tools/Compression.hpp"

#include <netinet/in.h>
#include <zlib.h>

namespace robot_remote_control {

TransportWrapperGzip::TransportWrapperGzip(std::shared_ptr<Transport> transport, const int &compressionlevel):TransportWrapper(transport), compressionlevel(compressionlevel) {}


int TransportWrapperGzip::send(const std::string& uncompressed, Transport::Flags flags) {
    std::string compressed;
    int32_t len = Compression::compressString(uncompressed, &compressed);
    if (transport->send(compressed, flags)) {
        return len;
    }
    return 0;
}

int TransportWrapperGzip::receive(std::string* uncompressed, Transport::Flags flags) {
    std::string compressed;
    if (transport->receive(&compressed, flags)) {
        int32_t len = Compression::decompressString(compressed, uncompressed);
        return len;
    }
    return 0;
}


}  // namespace robot_remote_control

