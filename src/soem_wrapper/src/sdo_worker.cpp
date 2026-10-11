//
// Created by hang on 10/11/26.
//
#include "soem_wrapper/sdo_worker.hpp"
#include "soem_wrapper/utils/logger_utils.hpp"

namespace aim::ecat {
    using namespace logging;

    SdoWorker::~SdoWorker() {
        stop();
    }

    void SdoWorker::start() {
        std::lock_guard lock(mtx_);
        if (running_) {
            return;
        }
        running_ = true;
        thread_ = std::thread(&SdoWorker::loop, this);
        RCLCPP_INFO(*get_sys_logger(), "SDO worker started");
    }

    void SdoWorker::stop() {
        size_t dropped;
        {
            std::lock_guard lock(mtx_);
            if (!running_) {
                return;
            }
            running_ = false;
            dropped = jobs_.size();
            jobs_.clear();
        }
        cv_.notify_all();
        // wait for the running job
        if (thread_.joinable()) {
            thread_.join();
        }
        RCLCPP_INFO(*get_sys_logger(), "SDO worker stopped, %zu pending job(s) dropped", dropped);
    }

    bool SdoWorker::submit(Job job) {
        {
            std::lock_guard lock(mtx_);
            if (!running_) {
                return false;
            }
            jobs_.push_back(std::move(job));
        }
        cv_.notify_one();
        return true;
    }

    void SdoWorker::loop() {
        std::unique_lock lock(mtx_);
        while (true) {
            cv_.wait(lock, [this] {
                return !running_ || !jobs_.empty();
            });
            if (!running_) {
                return;
            }
            Job job = std::move(jobs_.front());
            jobs_.pop_front();

            // the job blocks, don't hold the lock while running it
            lock.unlock();
            try {
                job();
            } catch (const std::exception &e) {
                RCLCPP_ERROR(*get_sys_logger(), "SDO worker job failed: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(*get_sys_logger(), "SDO worker job failed");
            }
            lock.lock();
        }
    }
}
