#pragma once

#include "UpdateThread/ThreadProtectedVar.hpp"
#include "RingBuffer.hpp"
#include "MessageTypes.hpp"

#include <memory>
#include <vector>

namespace robot_remote_control {


template <class BUFFERTYPE> class SimpleBuffer : public ThreadProtectedVar< std::vector< std::shared_ptr<RingBufferBase> > >{
    public:
        SimpleBuffer() {};

        virtual ~SimpleBuffer() {}

        void initBufferID (const uint16_t &id) {
            lock();

            if (get_ref().size() <= id) {
                // resize(id+1); //if id == 1, indeyx should be one, so we need size two
                get_ref().resize(id + 1);  // if id == 1, index should be one, so we need size two
            }

            if (get_ref()[id].get() == nullptr) {
                    std::shared_ptr<RingBufferBase> newbuf;
                    newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<BUFFERTYPE>(1));
                    get_ref()[id] = newbuf;
            }

            unlock();
        }


};

}  // namespace robot_remote_control