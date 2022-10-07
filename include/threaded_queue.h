#pragma once

#include <queue>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>

// Small threaded queue implementation, essentially a primitive 
template <typename T>
struct ThreadedQueue
{
    private:
        std::mutex mutex;
        std::condition_variable cond;
        std::queue<T> queue;

    public:

        ThreadedQueue() = default;
        ThreadedQueue(const ThreadedQueue &) = delete; // should not copy threaded queues
        ThreadedQueue& operator=(const ThreadedQueue &) = delete; // should not copy-assign threaded queues

        T pop() {
            std::unique_lock<std::mutex> lock(mutex);
            while (this->queue.empty()) {
                cond.wait(lock);
            }
            auto item = queue.front();
            queue.pop();
            return item;
        }

        T push(const T &item) {
            std::unique_lock<std::mutex> lock(mutex);
            queue.push(item);
            lock.unlock();
            cond.notify_one();
        }
};
