#pragma once

#include <thread>
#include <future>
#include <functional>
#include "../RobotController/RobotController.hpp"

namespace robot_remote_control {

/**
 * @brief Thread decoupling class for the controller.addTelemetryReceivedCallback() function
 * 
 * @tparam DATATYPE protibuf type
 */

template <class DATATYPE> class TelemetryCallbackThread {

 public:
    TelemetryCallbackThread(robot_remote_control::RobotController& controller, const MessageId &type, std::function<void (const DATATYPE&)> callback, const ChannelId &channel = 0, std::function<void ()> busyCallback = [](){} ) : cb(callback), busycb(busyCallback) {

        thread_finished = true;

        // set callback to trigger thread
        controller.addTelemetryReceivedCallback<DATATYPE>(type, [&](const DATATYPE & data) {
            // only start a new thread if the previous one has finished
            if (thread_finished) {
                thread_finished = false;
                // deep copy received data to have a copy the thread can work with
                threadData.CopyFrom(data);
                // create thread to call the callback in
                std::thread t = std::thread([&]() {
                    cb(threadData);
                    thread_finished = true;
                });
                // run in detached mode to return directly (std::thread destructor would wait for thread to finish)
                t.detach();
            } else {
                busycb();
            }
        }, channel);
    };
    virtual ~TelemetryCallbackThread(){};

    bool finished() {
        return thread_finished;
    }

 protected:
    DATATYPE threadData;
    std::function<void (const DATATYPE&)> cb;
    std::function<void ()> busycb;
    std::atomic<bool> thread_finished;
    std::mutex mutex;
};

}
