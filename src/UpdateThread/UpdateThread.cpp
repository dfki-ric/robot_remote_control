

#include "UpdateThread.hpp"

using namespace interaction;


UpdateThread::UpdateThread(){
    threadRunning.get() = false;
}

UpdateThread::~UpdateThread(){
    if (threadRunning.get()){
        stopUpdateThread();
    }
}

void UpdateThread::updateThreadMain(const unsigned int &milliseconds){
    while (threadRunning.get()){
        update();
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }
}

void UpdateThread::startUpdateThread(const unsigned int &milliseconds){
    threadRunning.get() = true;
    updateThread = std::thread(&UpdateThread::updateThreadMain, this, std::ref(milliseconds));
}

void UpdateThread::stopUpdateThread(){
    threadRunning.get() = false;
    updateThread.join();
}
