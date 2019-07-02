#pragma once

#include <mutex>

template <class C> class ThreadProtecetedVar{

    public:

        ThreadProtecetedVar(){};
        virtual ~ThreadProtecetedVar(){};

        C& get(){
            std::lock_guard<std::mutex> lock(mutex);
            return var;
        }


    private:
        C var;
        std::mutex mutex;

};