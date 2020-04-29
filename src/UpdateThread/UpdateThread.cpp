

#include "UpdateThread.hpp"
#include <unistd.h>
#include <utility>

using namespace robot_remote_control;


UpdateThread::UpdateThread():running(false) {
}

UpdateThread::~UpdateThread() {
    if (running) {
        stopUpdateThread();
    }
}

void UpdateThread::updateThreadMain(const unsigned int &milliseconds, std::future<void> runningFuture) {
    running = true;
    while (runningFuture.wait_for(std::chrono::milliseconds(milliseconds)) == std::future_status::timeout) {
        update();
    }
    running = false;
}

void UpdateThread::startUpdateThread(const unsigned int &milliseconds) {
    if (!running) {
        stopFuture = stopPromise.get_future();
        updateThread = std::thread(&UpdateThread::updateThreadMain, this, std::move(milliseconds), std::move(stopFuture));
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
