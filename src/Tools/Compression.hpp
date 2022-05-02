#pragma once

#include <string>

namespace robot_remote_control {

class Compression {
 public:
        static int32_t compressString(const std::string& uncompressed, std::string* compressed, const int &compressionlevel = -1);

        static int32_t decompressString(const std::string& compressed, std::string* uncompressed);

};

}  // namespace robot_remote_control

