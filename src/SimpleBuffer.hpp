#pragma once

#include "UpdateThread/AtomicClass.hpp"
#include "RingBuffer.hpp"
#include "MessageTypes.hpp"

#include <memory>
#include <vector>

namespace robot_remote_control {


template <class BUFFERTYPE> class SimpleBuffer : public AtomicClass< std::vector< std::shared_ptr<RingBufferBase> > >{
    public:
        SimpleBuffer() {};

        virtual ~SimpleBuffer() {}

        void initBufferID (const uint16_t &id) {
            auto lockedAccessObject = lockedAccess();
            if (lockedAccessObject.get().size() <= id) {
                // resize(id+1); //if id == 1, indeyx should be one, so we need size two
                lockedAccessObject.get().resize(id + 1);  // if id == 1, index should be one, so we need size two
            }

            if (lockedAccessObject.get()[id].get() == nullptr) {
                    std::shared_ptr<RingBufferBase> newbuf;
                    newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<BUFFERTYPE>(1));
                    lockedAccessObject.get()[id] = newbuf;
            }
        }
};

}  // namespace robot_remote_control