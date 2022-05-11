
#pragma once

#include <vector>
#include <memory>
#include <functional>
#include <algorithm>


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
        virtual size_t dropped() = 0;
        virtual void resize(const size_t &newsize) = 0;
        virtual void clear() = 0;
        virtual bool pop(bool onlyNewest = false) = 0;
};

/**
 * @brief implements a ring buffer for a specific type
 * 
 * TIP: Use pointers
 * 
 */

template <class TYPE> class RingBuffer: public RingBufferBase {
    public:
        explicit RingBuffer(const size_t & buffersize): RingBufferBase(), buffersize(buffersize), contentsize(0), in(0), out(0), droppedMessages(0) {
            buffer.resize(buffersize);
        }

        virtual ~RingBuffer() {}

        size_t size() {
            return contentsize;
        }

        size_t capacity() {
            return buffersize;
        }

        size_t dropped() {
            return droppedMessages;
        }

        void clear() {
            contentsize = 0;
            in = 0;
            out = 0;
        }

        /**
         * @brief resize the buffer
         * @warning buffer will be emptied
         * 
         * @param newsize the size of the new buffer
         */
        void resize(const size_t &newsize) {
            if (contentsize > 0) {
                clear();
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
                ++contentsize;
                ++in;
                in %= buffersize;
                notify(data);
                return true;
            } else if (overwriteIfFull) {
                // buffer full, force-push (overwrite latest)
                buffer[in] = data;
                ++out;
                out %= buffersize;
                ++in;
                in %= buffersize;
                notify(data);
                ++droppedMessages;
                return true;
            }
            ++droppedMessages;
            return false;
        }

        bool popData(TYPE *data, bool onlyNewest = false) {
            if (contentsize > 0) {
                if (onlyNewest) {
                    // move out pointer forward to last value
                    out += contentsize-1;
                    out %= buffersize;
                    contentsize = 1;
                }
                *data = buffer[out];
                --contentsize;
                ++out;
                out %= buffersize;
                return true;
            }
            return false;
        }

        bool pop(bool onlyNewest = false) {
            if (contentsize > 0) {
                if (onlyNewest) {
                    // move out pointer forward to last value
                    out += contentsize-1;
                    out %= buffersize;
                    contentsize = 1;
                }
                --contentsize;
                ++out;
                out %= buffersize;
                return true;
            }
        }

        bool peekData(TYPE *data) {
            if (contentsize > 0) {
                *data = buffer[out];
                return true;
            }
            return false;
        }

        void addDataReceivedCallback(const std::function<void(const TYPE & data)> &cb) {
            callbacks.push_back(cb);
        }

        void printState() {
            printf("%s : buffersize: %i, contentsize %i, in %i, out %i\n", typeid(*this).name(), buffersize, contentsize, in, out);
        }

    private:
        void notify(const TYPE & data) {
            auto callCb = [&](const std::function<void (const TYPE & data)> &cb){cb(data);};
            std::for_each(callbacks.begin(), callbacks.end(), callCb);
        }
        size_t buffersize, contentsize, in, out;
        size_t droppedMessages;
        std::vector<TYPE> buffer;

        std::vector< std::function<void (const TYPE & data)> > callbacks;
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

        template<class DATATYPE> static bool popData(std::shared_ptr<RingBufferBase> buffer, DATATYPE *data, bool onlyNewest = false) {
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            if (dataclass.get()) {
                return dataclass->popData(data, onlyNewest);
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

        template<class DATATYPE> static bool addDataReceivedCallback(const std::shared_ptr<RingBufferBase> &buffer, const std::function<void(const DATATYPE & data)> &cb) {
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            if (dataclass.get()) {
                dataclass->addDataReceivedCallback(cb);
                return true;
            }
            return false;
        }
};

}  // namespace robot_remote_control
