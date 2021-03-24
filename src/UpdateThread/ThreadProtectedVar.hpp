#pragma once

#include <mutex>
#include <memory>

namespace robot_remote_control {

/**
 * @brief Template class to make variables thread safe
 * 
 * @tparam C The class type to have thread safe
 * 
 * Examples:
 *  ThreadProtectedVar< std::vector<int> > thread_int;
 *  thread_int->lockedAccess()->push_back(5);
 *  //for long-term locks
 *  { // extra scope to release the lock
 *      auto lockedAccessObject = thread_int.lockedAccess();
 *      for (int i = 0; i<5; ++i) {
 *          lockedAccessObject->push_back(i);
 *      }
 *      int myint = lockedAccessObject.get()[3];
 *  }
 *
 *  int myint2 = thread_int->lockedAccess().get()[5];
 *  int size = thread_int->lockedAccess()->size();
 */
template <class C> class ThreadProtectedVar{
    public:
        class LockedAccess {            
         public:
            LockedAccess(std::mutex *mutex, C *object): accesslock(*mutex), object(object) { }

            C* operator->() {
                return object;
            }
            C& get() {
                return *object;
            }
            void set(const C& value) {
                *object = value;
            }
         private:
            std::unique_lock<std::mutex> accesslock;
            C *object;
        };

        ThreadProtectedVar() {}
        explicit ThreadProtectedVar(const C &init):var(init) {}

        virtual ~ThreadProtectedVar() {}

        /**
         * @brief Get the protected variable as access object allowing direct, locked access
         * 
         * @return LockedAccess object that locked the mutex as log it is in scope
         */
        LockedAccess lockedAccess() {
            return {&mutex, &var};
        }

        // no copy constructors
        ThreadProtectedVar(const ThreadProtectedVar&) = delete;
        ThreadProtectedVar& operator=(const ThreadProtectedVar&) = delete;

    private:
        C var;
        std::mutex mutex;
};

}  // namespace robot_remote_control
