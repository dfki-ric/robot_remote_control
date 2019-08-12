#pragma once

#include "UpdateThread/ThreadProtectedVar.hpp"
#include "RingBuffer.hpp"

namespace controlledRobot
{

class TelemetryBuffer: public ThreadProtecetedVar< std::vector< std::shared_ptr<RingBufferBase> > >{
    public:
    
    TelemetryBuffer(const size_t &size = 1);

    ~TelemetryBuffer();


    

};

}