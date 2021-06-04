
#pragma once

#include <vector>
#include <memory>
#include <functional>


namespace robot_remote_control {

/**
 * @brief base class to be able to hold buffers of different types in a vector 
 */
class RingBufferBase{
    public:
        RingBufferBase() {}
        virtual ~RingBufferBase() {}

        virtual size_t size() = 0;
        virtual size_t capacity() = 0;
        virtual void resize(const size_t &newsize) = 0;
        // virtual void clear();
};

/**
 * @brief implements a ring buffer for a specific type
 * 
 * TIP: Use pointers
 * 
 */

template <class TYPE> class RingBuffer: public RingBufferBase {
    public:
        explicit RingBuffer(const size_t & buffersize): RingBufferBase(), buffersize(buffersize), contentsize(0), in(0), out(0) {
            buffer.resize(buffersize);
        }

        virtual ~RingBuffer() {}

        size_t size() {
            return contentsize;
        }

        size_t capacity() {
            return buffersize;
        }

        void resize(const size_t &newsize) {
            if (contentsize > 0) {
                printf("possible data loss due to resize on nonemty buffer\n");
            }

            buffersize = newsize;
            buffer.resize(buffersize);

            if (in > buffersize) {
                in = buffersize;
            }

            if (out > buffersize) {
                out = 0;
            }
        }

        bool pushData(const TYPE & data, bool overwriteIfFull = false) {
            if (contentsize != buffersize) {
                buffer[in] = data;
                contentsize++;
                in++;
                in %= buffersize;
                notify(data);
                return true;
            } else if (overwriteIfFull) {
                // buffer full, force-push (overwrite latest)
                buffer[in] = data;
                out++;
                out %= buffersize;
                in++;
                in %= buffersize;
                notify(data);
                return true;
            }
            return false;
        }

        bool popData(TYPE *data) {
            if (contentsize > 0) {
                *data = buffer[out];
                contentsize--;
                out++;
                out %= buffersize;
                return true;
            }
            return false;
        }

        bool peekData(TYPE *data) {
            if (contentsize > 0) {
                *data = buffer[out];
                return true;
            }
            return false;
        }

        bool addDataReceivedCallback(const std::function<void(const size_t& buffersize, const TYPE & data)> &cb) {
            callbacks.push_back(cb);
        }

    private:
        void notify(const TYPE & data) {
            for (auto& cb : callbacks) {
                cb(size(), data);
            }
        }
        size_t buffersize, contentsize, in, out;
        std::vector<TYPE> buffer;

        std::vector< std::function<void (const size_t& buffersize, const TYPE & data)> > callbacks;
};


class RingBufferAccess{
    public:
        template<class DATATYPE> static bool pushData(std::shared_ptr<RingBufferBase> buffer, const DATATYPE & data, bool overwrite = false) {
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            if (dataclass.get()) {
                return dataclass->pushData(data, overwrite);
            }
            return false;
        }

        template<class DATATYPE> static bool popData(std::shared_ptr<RingBufferBase> buffer, DATATYPE *data) {
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            if (dataclass.get()) {
                return dataclass->popData(data);
            }
            return false;
        }

        template<class DATATYPE> static bool peekData(const std::shared_ptr<RingBufferBase> &buffer, DATATYPE *data) {
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            if (dataclass.get()) {
                return dataclass->peekData(data);
            }
            return false;
        }

        template<class DATATYPE> static bool addDataReceivedCallback(const std::shared_ptr<RingBufferBase> &buffer, const std::function<void(const size_t& buffersize, const DATATYPE & data)> &cb) {
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            if (dataclass.get()) {
                return dataclass->addDataReceivedCallback(cb);
            }
            return false;
        }
};

}  // namespace robot_remote_control
