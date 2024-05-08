#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

template <class T>
class BlockingQueue
{
public:
    void push(T item)
    {
        std::lock_guard guard(lock);
        queue.push_back(std::move(item));
        signal_added.notify_one();
    }

    T pop()
    {
        while (true) {
            std::unique_lock guard(lock);
            if (!queue.empty()) {
                T pop = std::move(queue.front());
                queue.pop_front();
                return pop;
            }
            signal_added.wait(guard);
        }
    }

private:
    std::deque<T> queue;

    std::condition_variable signal_added;
    std::mutex lock;
};