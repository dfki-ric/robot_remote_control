#pragma once

#include <mutex>
#include <memory>

namespace robot_remote_control
{

/**
 * @brief Template class to make variables thread save
 * 
 * @tparam C The class type to have thread save
 */
template <class C> class ThreadProtectedVar{

    public:

        template <class REF> class LockedAccess {
         public:
            LockedAccess(ThreadProtectedVar<C> *parent, REF &reference): parent(parent), reference(reference) {
                parent->lock();
            }
            ~LockedAccess() {
                parent->unlock();
            }
            REF* operator->() {
                return &reference;
            }
            REF& operator*() {
                return reference;
            }
            REF& get() {
                return reference;
            }
         private:
            ThreadProtectedVar<C> *parent;
            REF &reference;
        };

        ThreadProtectedVar():
            islocked(false)
        {};

        ThreadProtectedVar(const C &init):
            var(init),
            islocked(false)
        {};

        // no copy constructors
        ThreadProtectedVar(const ThreadProtectedVar&) = delete;
        ThreadProtectedVar& operator=(const ThreadProtectedVar&) = delete;

        virtual ~ThreadProtectedVar(){};

        /**
         * @brief get a copy of the contained variable
         * 
         * @return const C a copy of the protected variable
         */
        const C get(){
            std::lock_guard<std::mutex> lock(mutex);
            C returnvar = var;
            return returnvar;
        }

        const C operator()(){
            return get();
        }

        /**
         * @brief set the protected variable
         * 
         * @param val the new value
         */
        void set(const C &val){
            std::lock_guard<std::mutex> lock(mutex);
            var = val;
        }

        /**
         * @brief Get the protected variable as reference allowing direct access
         * 
         * @return C& reference to the protected variable
         */
        LockedAccess<C> get_ref() {
            return LockedAccess<C>(this, var);
        }

        C& operator=(const C& other) {
            std::lock_guard<std::mutex> lock(mutex);
            var = other;
            return *this;
        }
    private:

        /**
         * @brief lock the protected variable manually to allow a reference access
         */
        void lock(){
            mutex.lock();
        }

        /**
         * @brief unlock the protected variable after it was manually locked
         */
        void unlock(){
            mutex.unlock();
        }

        C var;
        bool islocked;
        std::mutex mutex;
};

}
