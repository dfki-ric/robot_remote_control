
#include "Compression.hpp"

#include <zlib.h>
#include <netinet/in.h>

namespace robot_remote_control {

int32_t Compression::compressString(const std::string& uncompressed, std::string* compressed, const int &compressionlevel) {
    if (uncompressed.size() == 0) {
        compressed->clear();
        return 0;
    }
    uLong destLen = compressBound(uncompressed.size());
    uLong srcLen = uncompressed.size();
    if (compressed->size() < destLen) {
        // set min size for compression
        compressed->resize(destLen + sizeof(uint32_t));
    }
    Byte* compressedPtr = const_cast<Byte*>(reinterpret_cast<const Byte*>(compressed->data())) + sizeof(uint32_t);
    const Byte* sourcePtr = reinterpret_cast<const Byte*>(uncompressed.data());
    int res = compress2(compressedPtr, &destLen, sourcePtr, srcLen, compressionlevel);

    // not optimal, might reallocate memory
    // set real size after compression
    compressed->resize(destLen + sizeof(uint32_t));
    uint32_t* uncompressedSize = const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(compressed->data()));
    *uncompressedSize = htonl(uncompressed.size());
    return srcLen;
}

int32_t Compression::decompressString(const std::string& compressed, std::string* uncompressed) {
    const uint32_t* uncompressedSizePtr = reinterpret_cast<const uint32_t*>(compressed.data());
    uLong uncompressedSize = ntohl(*uncompressedSizePtr);
    uLong srcLen = compressed.size()-sizeof(uint32_t);
    if (uncompressed->size() < uncompressedSize) {
        uncompressed->resize(uncompressedSize);
    }
    Byte* data = const_cast<Byte*>(reinterpret_cast<const Byte*>(uncompressed->data()));
    const Byte* sourcePtr = reinterpret_cast<const Byte*>(compressed.data()) + sizeof(uint32_t);
    int res = uncompress(data, &uncompressedSize, sourcePtr, srcLen);
    uncompressed->resize(uncompressedSize);
    return uncompressed->size();
}


}  // namespace robot_remote_control
