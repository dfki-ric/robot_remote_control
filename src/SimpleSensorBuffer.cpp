#include "SimpleSensorBuffer.hpp"


namespace robot_remote_control {


SimpleSensorBuffer::SimpleSensorBuffer() {}


// void SimpleSensorBuffer::resize(const size_t &size){
//         // lock();
//         //resize the vector
//         get_ref().resize(size);
//         for (unsigned int i = 0;i<size;i++){
//             if (get_ref()[i].get() == nullptr){
//                 std::shared_ptr<RingBufferBase> newbuf;
//                 newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<SimpleSensor>(1));
//                 get_ref()[i] = newbuf;
//             }
//         }
//         // unlock();
// }

void SimpleSensorBuffer::initBufferID(const uint16_t &id) {
        auto lockObject = get_ref();

        if (lockObject->size() <= id) {
            // resize(id+1); //if id == 1, indeyx should be one, so we need size two
            lockObject->resize(id + 1);  // if id == 1, index should be one, so we need size two
        }

        if (lockObject->[id].get() == nullptr) {
                std::shared_ptr<RingBufferBase> newbuf;
                newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<SimpleSensor>(1));
                lockObject->[id] = newbuf;
        }
}

}  // namespace robot_remote_control
