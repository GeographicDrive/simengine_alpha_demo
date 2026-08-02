// simengine/core/job_system.hpp — Core Engine subsystem.
//
// Simple persistent thread-pool job system. Threads are spawned once at
// construction and parked on a condition variable between jobs — no
// thread creation/destruction during the simulation loop, consistent
// with the "no allocation/no heavyweight syscalls in the hot loop"
// invariant. Provides a `parallelFor` helper for the common case (batch
// operations over dense component arrays, e.g. integrating N rigid
// bodies), plus raw `submit`/`waitAll` for arbitrary tasks (background
// asset streaming, terrain tile generation, etc.).
//
// This is intentionally a straightforward queue+mutex pool, not a
// work-stealing scheduler — correct and simple first; a work-stealing
// version is a drop-in replacement later if profiling shows contention,
// without changing this module's public interface.

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace simengine::core {

class JobSystem {
public:
    explicit JobSystem(unsigned threadCount = 0) {
        if (threadCount == 0) {
            threadCount = std::max(1u, std::thread::hardware_concurrency());
        }
        workers_.reserve(threadCount);
        for (unsigned i = 0; i < threadCount; ++i) {
            workers_.emplace_back([this] { workerLoop(); });
        }
    }

    ~JobSystem() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        cv_.notify_all();
        for (auto& t : workers_) if (t.joinable()) t.join();
    }

    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    void submit(std::function<void()> job) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++pending_;
            queue_.push(std::move(job));
        }
        cv_.notify_one();
    }

    // Blocks the calling thread until every job submitted so far has
    // completed. Safe to call from the main/sim thread between frames.
    void waitAll() {
        std::unique_lock<std::mutex> lock(mutex_);
        doneCv_.wait(lock, [this] { return pending_ == 0; });
    }

    // Splits [0, count) into contiguous ranges across the pool and runs
    // fn(begin, end) on each, blocking until all ranges complete. This is
    // the primary entry point systems should use for batch work over
    // ComponentStorage<T>::dense() arrays.
    void parallelFor(std::size_t count, std::size_t minGrainSize,
                      const std::function<void(std::size_t begin, std::size_t end)>& fn) {
        if (count == 0) return;
        const std::size_t workerCount = std::max<std::size_t>(1, workers_.size());
        const std::size_t grain = std::max(minGrainSize, (count + workerCount - 1) / workerCount);

        for (std::size_t begin = 0; begin < count; begin += grain) {
            const std::size_t end = std::min(count, begin + grain);
            submit([fn, begin, end] { fn(begin, end); });
        }
        waitAll();
    }

    unsigned threadCount() const noexcept { return static_cast<unsigned>(workers_.size()); }

private:
    void workerLoop() {
        for (;;) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
                if (stopping_ && queue_.empty()) return;
                job = std::move(queue_.front());
                queue_.pop();
            }
            job();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                --pending_;
                if (pending_ == 0) doneCv_.notify_all();
            }
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable doneCv_;
    std::size_t pending_ = 0;
    bool stopping_ = false;
};

} // namespace simengine::core
