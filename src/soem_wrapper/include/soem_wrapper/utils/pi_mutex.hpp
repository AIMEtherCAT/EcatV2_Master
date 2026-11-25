//
// Created by hang on 10/11/26.
//

#ifndef SOEM_WRAPPER_UTILS_PI_MUTEX_HPP
#define SOEM_WRAPPER_UTILS_PI_MUTEX_HPP

#include <pthread.h>

namespace aim::utils {
    /**
     * mutex with priority inheritance (PTHREAD_PRIO_INHERIT), usable with std::lock_guard / std::unique_lock
     *
     * used for the locks shared between the realtime data thread and non-realtime threads (executor, sdo worker),
     * a non-realtime thread holding the lock is boosted to the priority of the realtime thread waiting for it,
     * so it can't be preempted by other normal threads while the realtime thread is blocked (priority inversion).
     * the same as the locks of the soem nic driver
     */
    class PiMutex {
    public:
        PiMutex() {
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
            pthread_mutex_init(&mutex_, &attr);
            pthread_mutexattr_destroy(&attr);
        }

        ~PiMutex() {
            pthread_mutex_destroy(&mutex_);
        }

        PiMutex(const PiMutex &) = delete;

        PiMutex &operator=(const PiMutex &) = delete;

        void lock() {
            pthread_mutex_lock(&mutex_);
        }

        bool try_lock() {
            return pthread_mutex_trylock(&mutex_) == 0;
        }

        void unlock() {
            pthread_mutex_unlock(&mutex_);
        }

    private:
        pthread_mutex_t mutex_{};
    };
}

#endif //SOEM_WRAPPER_UTILS_PI_MUTEX_HPP
