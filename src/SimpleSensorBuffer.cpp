#include "SimpleSensorBuffer.hpp"


using namespace robot_remote_control;


SimpleSensorBuffer::SimpleSensorBuffer(){}


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

void SimpleSensorBuffer::initBufferID (const unsigned int &id){

        lock();
    
        if (get_ref().size() <= id){
            //resize(id+1); //if id == 1, indeyx should be one, so we need size two
            get_ref().resize(id+1);//if id == 1, index should be one, so we need size two
        }
            
        if (get_ref()[id].get() == nullptr){
                std::shared_ptr<RingBufferBase> newbuf;
                newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<SimpleSensor>(1));
                get_ref()[id] = newbuf;
        }

        unlock();
}