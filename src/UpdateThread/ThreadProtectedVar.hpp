#pragma once

#include <mutex>

template <class C> class ThreadProtecetedVar{

    public:

        ThreadProtecetedVar():
            islocked(false)
        {
        };
        virtual ~ThreadProtecetedVar(){};

        const C get(){
            //std::lock_guard<std::mutex> lock(mutex);
            mutex.lock();
            C returnvar = var;
            mutex.unlock();
            return returnvar;
        }

        void set(const C &val){
            std::lock_guard<std::mutex> lock(mutex);
            var = val;
        }

        void lock(){
            mutex.lock();
            islocked = true;
        }

        void unlock(){
            islocked = false;
            mutex.unlock();
        }

        C& get_ref(){
            if (!islocked){
                throw std::runtime_error("ThreadProtecetedVar must be locked before get_ref() is used");
            }
            return var;
        }
        
    private:
        C var;
        bool islocked;
        std::mutex mutex;

};