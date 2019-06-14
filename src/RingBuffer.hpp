#pragma once

#include <vector>

/**
 * @brief implements a ring buffer for a specific type
 * 
 * TIP: Use pointers
 * 
 */
template <class TYPE> class RingBuffer{
    public:
        
        RingBuffer(const unsigned int & buffersize):buffersize(buffersize),contentsize(0),in(0),out(0){
            buffer.resize(buffersize);
        }

        unsigned int size(){
            return contentsize;
        }


        bool pushData(const TYPE & data){
            if (contentsize != buffersize){
                buffer[in] = data;
                contentsize++;
                in++;
                in %= buffersize;
                return true;
            }
            printf("Buffer full\n");
            return false;
        }

        bool popData(TYPE& data){
            if (contentsize>0){
                data = buffer[out];
                contentsize--;
                out++;
                out %= buffersize;
                return true;
            }
            printf("buffer empty\n");
            return false;
        }

    private:
        unsigned int buffersize,contentsize,in,out;
        
        std::vector<TYPE> buffer;

};