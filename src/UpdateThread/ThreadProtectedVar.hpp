#pragma once

#include <mutex>
#include <memory>

namespace robot_remote_control {

/**
 * @brief Template class to make variables thread safe
 * 
 * @tparam C The class type to have thread safe
 */
template <class C> class ThreadProtectedVar{
    public:
        class LockedAccess {
         private:
            friend class ThreadProtectedVar;
            LockedAccess(std::mutex *mutex, C *reference): parent(parent), reference(reference) {
                lock = std::make_shared< std::lock_guard<std::mutex> >(*mutex);
            }
         public:
            ~LockedAccess() {}

            C* operator->() {
                return reference;
            }
            C& get() {
                return *reference;
            }
         private:
            std::shared_ptr< std::lock_guard<std::mutex> >lock;
            ThreadProtectedVar<C> *parent;
            C *reference;
        };

        ThreadProtectedVar() {}
        explicit ThreadProtectedVar(const C &init):var(init) {}

        virtual ~ThreadProtectedVar() {}

        // no copy constructors
        ThreadProtectedVar(const ThreadProtectedVar&) = delete;
        ThreadProtectedVar& operator=(const ThreadProtectedVar&) = delete;

        /**
         * @brief get a copy of the contained variable
         * 
         * @return const C a copy of the protected variable
         */
        const C get() {
            std::lock_guard<std::mutex> lock(mutex);
            C returnvar = var;
            return returnvar;
        }

        const C operator()() {
            return get();
        }

        /**
         * @brief set the protected variable
         * 
         * @param val the new value
         */
        void set(const C &val) {
            std::lock_guard<std::mutex> lock(mutex);
            var = val;
        }

        /**
         * @brief Get the protected variable as reference allowing direct access
         * 
         * @return C& reference to the protected variable
         */
        LockedAccess getLockedAccess() {
            return LockedAccess(&mutex, &var);
        }

        C& operator=(const C& other) {
            std::lock_guard<std::mutex> lock(mutex);
            var = other;
            return *this;
        }
    private:
        C var;
        std::mutex mutex;
};

}  // namespace robot_remote_control
