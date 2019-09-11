#pragma once

#include <thread>
#include <chrono>
#include <future>

#include "ThreadProtectedVar.hpp"

namespace controlledRobot
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

        private:
            std::thread updateThread;
            std::promise<void> stopPromise;
            std::future<void> stopFuture;
            void updateThreadMain(const unsigned int &milliseconds, std::future<void> runningFuture);
            bool running;

};

}