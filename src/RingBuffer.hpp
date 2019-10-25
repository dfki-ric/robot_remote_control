#pragma once

#include <vector>
#include <memory>

/**
 * @brief base class to be able to hold buffers of different types in a vector 
 */
class RingBufferBase{
    public:
        RingBufferBase(){};
        virtual ~RingBufferBase(){};

        virtual unsigned int size() = 0;
        virtual unsigned int capacity() = 0;
        virtual void resize(const unsigned int &newsize) = 0;
        //virtual void clear();
};

/**
 * @brief implements a ring buffer for a specific type
 * 
 * TIP: Use pointers
 * 
 */

template <class TYPE> class RingBuffer: public RingBufferBase{
    public:
        
        RingBuffer(const unsigned int & buffersize):RingBufferBase(),buffersize(buffersize),contentsize(0),in(0),out(0){
            buffer.resize(buffersize);
        }

        virtual ~RingBuffer(){}

        unsigned int size(){
            return contentsize;
        }

        unsigned int capacity(){
            return buffersize;
        }

        void resize(const unsigned int &newsize){

            if (contentsize > 0){
                printf("possible data loss due to resize on nonemty buffer\n");
            }

            buffersize = newsize;
            buffer.resize(buffersize);

            if (in > buffersize){
                in = buffersize;
            }

            if (out > buffersize){
                out = 0;
            }

        }

        bool pushData(const TYPE & data){
            if (contentsize != buffersize){
                buffer[in] = data;
                contentsize++;
                in++;
                in %= buffersize;
                return true;
            }else{
                //buffer full, force-push (overwrite latest)
                buffer[in] = data;
                out++;
                out %= buffersize;
                in++;
                in %= buffersize;
            }
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
            return false;
        }

        bool peekData(TYPE& data){
            if (contentsize>0){
                data = buffer[out];
                return true;
            }
            return false;
        }

    private:
        unsigned int buffersize,contentsize,in,out;
        
        std::vector<TYPE> buffer;

};


class RingBufferAccess{
    public:
        
        template<class DATATYPE> static bool pushData(std::shared_ptr<RingBufferBase> &buffer, const DATATYPE & data){
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            return dataclass->pushData(data);
        }

        template<class DATATYPE> static bool popData(std::shared_ptr<RingBufferBase> &buffer, DATATYPE & data){
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            return dataclass->popData(data);
        }

        template<class DATATYPE> static bool peekData(std::shared_ptr<RingBufferBase> &buffer, DATATYPE & data){
            std::shared_ptr< RingBuffer<DATATYPE> > dataclass = std::dynamic_pointer_cast< RingBuffer<DATATYPE> >(buffer);
            return dataclass->peekData(data);
        }
};
