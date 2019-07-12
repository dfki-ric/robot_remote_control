#pragma once

#include <thread>
#include <chrono>
#include <future>

#include "ThreadProtectedVar.hpp"

namespace interaction
{

class UpdateThread{
    public: 

        UpdateThread();
        virtual ~UpdateThread();

        virtual void update() = 0;

        void startUpdateThread(const unsigned int &milliseconds);
        void stopUpdateThread();

        private:
            std::thread updateThread;
            std::promise<void> stopPromise;
            std::future<void> stopFuture;
            //ThreadProtecetedVar<bool> threadRunning;
            //ThreadProtecetedVar<bool> threadFinished;
            void updateThreadMain(const unsigned int &milliseconds, std::future<void> runningFuture);

};

}