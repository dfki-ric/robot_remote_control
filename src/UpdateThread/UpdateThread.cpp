

#include "UpdateThread.hpp"
#include <unistd.h>

using namespace controlledRobot;


UpdateThread::UpdateThread(){
}

UpdateThread::~UpdateThread(){
}

void UpdateThread::updateThreadMain(const unsigned int &milliseconds, std::future<void> runningFuture){
    while (runningFuture.wait_for(std::chrono::milliseconds(milliseconds)) == std::future_status::timeout){
        update();
    }
}

void UpdateThread::startUpdateThread(const unsigned int &milliseconds){
    stopFuture = stopPromise.get_future();
    updateThread = std::thread(&UpdateThread::updateThreadMain, this, std::move(milliseconds), std::move(stopFuture));
}

void UpdateThread::stopUpdateThread(){
    stopPromise.set_value();    
    if (updateThread.joinable()){
        updateThread.join();
    }
}
