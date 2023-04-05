

#include "UpdateThread.hpp"
#include <unistd.h>
#include <utility>

using namespace robot_remote_control;


UpdateThread::UpdateThread():running(false), threadTimer(std::make_shared<LockableClass<Timer>>()) {
}

UpdateThread::~UpdateThread() {
    if (running) {
        stopUpdateThread();
    }
}

void UpdateThread::updateThreadMain(const unsigned int &milliseconds, std::future<void> runningFuture, std::shared_ptr< LockableClass<Timer> > timer) {
    running = true;
    while (runningFuture.wait_for(std::chrono::milliseconds(milliseconds)) == std::future_status::timeout) {
        update();
        timer->lockedAccess()->start();
    }
    running = false;
}

void UpdateThread::startUpdateThread(const unsigned int &milliseconds) {
    if (!running) {
        stopFuture = stopPromise.get_future();
        updateThread = std::thread(&UpdateThread::updateThreadMain, this, std::move(milliseconds), std::move(stopFuture), threadTimer);
        running = true;
    }
}

void UpdateThread::stopUpdateThread() {
    if (running) {
        stopPromise.set_value();
        if (updateThread.joinable()) {
            updateThread.join();
        }
        running = false;
    }
}

float UpdateThread::getElapsedTimeInS() {
    float time = threadTimer->lockedAccess()->getElapsedTime();
    return time;
}

#ifdef USES_PTHREAD
bool UpdateThread::setUpdateThreadPriority(const int &priority, const int &policy){
    thread_params.sched_priority = priority;
    if(pthread_setschedparam(updateThread.native_handle(), policy, &thread_params)) {
        return false;
    }
    return true;
}
#endif
