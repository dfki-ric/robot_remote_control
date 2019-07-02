#pragma once

#include <thread>
#include <chrono>
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
            ThreadProtecetedVar<bool> threadRunning;
            void updateThreadMain(const unsigned int &milliseconds);

};

}