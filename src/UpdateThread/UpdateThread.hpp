#pragma once

#include <thread>
#include <chrono>
#include <future>
#include <memory>

#include "Timer.hpp"
#include "LockableClass.hpp"

#ifdef USES_PTHREAD
   #include <pthread.h>
#endif

namespace robot_remote_control
{

/**
 * @brief base class to have a threaded update by repeated calls to the virtual update() function
 */
class UpdateThread{
 public: 
    UpdateThread();
    virtual ~UpdateThread();

    /**
     * @brief update function to be implemented by inheriting classes
     * 
     */
    virtual void update() = 0;

    /**
     * @brief starts the thread
     * 
     * @param milliseconds milliseconds to wait after update() finishes
     * @TODO make this ms between calls to update()
     */
    void startUpdateThread(const unsigned int &milliseconds);

    /**
     * @brief waits until update() retruns and stops the thread
     * 
     */
    void stopUpdateThread();

    bool threaded(){
        return running;
    };

#ifdef USES_PTHREAD
    /**
     * @brief Set threading prioritied for the thread
     * Only available when using pthread
     * 
     * @param policy The pthread thread policy to use, e.g. SCHED_OTHER, SCHED_FIFO, or SCHED_RR
     * @param priority SCHED_FIFO, SCHED_RR (0-99), SCHED_OTHER, SCHED_IDLE, SCHED_BATCH (must be 0)  https://man7.org/linux/man-pages/man7/sched.7.html
     * @return true 
     * @return false 
     */
     bool setUpdateThreadPriority(const int &priority = 0, const int &policy = SCHED_OTHER);
#endif

 protected:
    /**
     * @brief Get the Elapsed Time In Seconds
     * Can be used in the update loop to check the time since the end of last update() call
     * 
     * @return float seconds elapsed since end of last loop
     */
    float getElapsedTimeInS();


 private:
    std::thread updateThread;
    std::promise<void> stopPromise;
    std::future<void> stopFuture;
    std::shared_ptr< LockableClass<Timer> > threadTimer;
    void updateThreadMain(const unsigned int &milliseconds, std::future<void> runningFuture, std::shared_ptr< LockableClass<Timer> > timer);
    bool running;
    sched_param thread_params;
};

}