#include "TransportWrapperGzip.hpp"

#include <netinet/in.h>
#include <zlib.h>

// TODO use protobuf size


namespace robot_remote_control {

TransportWrapperGzip::TransportWrapperGzip(std::shared_ptr<Transport> transport, const int &compressionlevel):transport(transport), compressionlevel(compressionlevel) {}


int TransportWrapperGzip::send(const std::string& uncompressed, Flags flags) {
    std::string compressed;
    uLong destLen = compressBound(uncompressed.size());
    uLong srcLen = uncompressed.size();
    if (compressed.size() < destLen) {
        // set min size for compression
        compressed.resize(destLen + sizeof(uint32_t));
    }
    Byte* compressedPtr = const_cast<Byte*>(reinterpret_cast<const Byte*>(compressed.data())) + sizeof(uint32_t);
    const Byte* sourcePtr = reinterpret_cast<const Byte*>(uncompressed.data());
    int res = compress2(compressedPtr, &destLen, sourcePtr, srcLen, compressionlevel);

    // not optimal, might reallocate memory
    // set real size after compression
    compressed.resize(destLen + sizeof(uint32_t));

    uint32_t* uncompressedSize = const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(compressed.data()));
    *uncompressedSize = htonl(uncompressed.size());

    return transport->send(compressed, flags);
}

int TransportWrapperGzip::receive(std::string* uncompressed, Flags flags) {
    std::string compressed;
    transport->receive(&compressed);

    const uint32_t* uncompressedSizePtr = reinterpret_cast<const uint32_t*>(compressed.data());
    uLong uncompressedSize = ntohl(*uncompressedSizePtr);
    uLong srcLen = compressed.size()-sizeof(uint32_t);
    if (uncompressed->size() < uncompressedSize) {
        uncompressed->resize(uncompressedSize);
    }
    Byte* data = const_cast<Byte*>(reinterpret_cast<const Byte*>(uncompressed->data()));
    const Byte* sourcePtr = reinterpret_cast<const Byte*>(compressed.data()) + sizeof(uint32_t);
    int res = uncompress(data, &uncompressedSize, sourcePtr, srcLen);
    return res;
}


}  // namespace robot_remote_control

