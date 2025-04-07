#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

/**
 * @brief A thread-safe queue for inter-thread communication.
 *
 * This template class provides a thread-safe interface to a queue,
 * using a mutex and a condition variable to protect access.
 *
 * @tparam T The type of elements stored in the queue.
 */
template<typename T>
class ThreadQueue {
public:
    /**
     * @brief Constructs a new ThreadQueue object.
     */
    ThreadQueue() = default;

    /**
     * @brief Destroys the ThreadQueue object.
     */
    ~ThreadQueue() = default;

    /**
     * @brief Pushes an element into the queue.
     *
     * This function acquires a lock, pushes the element into the queue,
     * and then notifies one waiting thread that a new element is available.
     *
     * @param value The element to be pushed.
     */
    void push(const T& value) {
        // Lock is acquired when lock_guard is constructed.
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(value);
        m_condVar.notify_one();
        // Lock is automatically released when lock goes out of scope.
    }

    /**
     * @brief Attempts to pop an element from the queue without blocking.
     *
     * This method returns immediately. If the queue is empty, it returns false.
     * Otherwise, it retrieves and removes the front element.
     *
     * @param result Reference to store the popped element.
     * @return true if an element was popped; false if the queue was empty.
     */
    bool try_pop(T& result) {
        // could use std::optional
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_queue.empty())
            return false;
        result = m_queue.front();
        m_queue.pop();
        return true;
    }

    /**
     * @brief Waits until an element is available and then pops it.
     *
     * This function blocks until an element is available in the queue.
     * It uses a condition variable to wait, rechecking the condition upon wake-up.
     *
     * @param result Reference to store the popped element.
     */
    void wait_and_pop(T& result) {
        std::unique_lock<std::mutex> lock(m_mutex);
        // Wait until the queue is not empty.
        m_condVar.wait(lock, [this]{ return !m_queue.empty(); });
        result = m_queue.front();
        m_queue.pop();            
    }

    /**
     * @brief Checks whether the queue is empty.
     *
     * This function acquires a lock to safely check the state of the queue.
     *
     * @return true if the queue is empty, false otherwise.
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    /**
     * @brief Resets the queue to an empty state.
     */
    void clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::queue<T> empty;
        std::swap(m_queue, empty);
    }

private:
    std::queue<T> m_queue;             ///< Underlying standard queue holding the data.
    mutable std::mutex m_mutex;          ///< Mutex to protect the queue.
    std::condition_variable m_condVar;   ///< Condition variable for managing waits.
};
