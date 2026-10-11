//
// Created by hang on 10/11/26.
//

#ifndef SOEM_WRAPPER_SDO_WORKER_HPP
#define SOEM_WRAPPER_SDO_WORKER_HPP

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace aim::ecat {
    /**
     * global worker for blocking mailbox (sdo) operations at runtime, it is not realtime.
     */
    class SdoWorker {
    public:
        using Job = std::function<void()>;

        SdoWorker() = default;

        ~SdoWorker();

        SdoWorker(const SdoWorker &) = delete;

        SdoWorker &operator=(const SdoWorker &) = delete;

        void start();

        void stop();

        /**
         * thread-safe, the job runs in the worker thread
         * @return false if the worker is not running, the job is dropped
         */
        bool submit(Job job);

    private:
        void loop();

        std::mutex mtx_;
        std::condition_variable cv_;
        std::deque<Job> jobs_{};
        bool running_ = false;
        std::thread thread_{};
    };
}

#endif //SOEM_WRAPPER_SDO_WORKER_HPP
