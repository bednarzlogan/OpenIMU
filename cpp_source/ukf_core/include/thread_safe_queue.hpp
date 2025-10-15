#pragma once

#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <mutex>

/**
 * @brief A thread-safe queue for inter-thread communication.
 *
 * This template class provides a thread-safe interface to a queue,
 * using a mutex and a condition variable to protect access.
 *
 * @tparam T The type of elements stored in the queue.
 */
template <typename T> class ThreadQueue {
public:
  /**
   * @brief Constructs a new ThreadQueue object.
   */
  ThreadQueue(size_t maxSize = 1000, bool dropOldest = true) {
    // Default constructor initializes an empty queue.
    // Optionally, you can set a maximum size or other parameters here.
    std::lock_guard<std::mutex> lock(m_mutex);
    m_maxSize = maxSize;
    m_dropOldest = dropOldest;
    m_queue.set_capacity(maxSize);
  };

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
  void push(const T &value) {
    // NOTE: we could change this method to accept raw T types so that
    // std move can be used to avoid copying
    // Lock is acquired when lock_guard is constructed.
    std::unique_lock<std::mutex> lock(m_mutex);

    if (!m_dropOldest) {
      m_condVar.wait(lock, [this] { return !m_queue.full(); });
    }

    if (m_queue.full()) {
      ++dropped_count_;
    }

    m_queue.push_back(value);
    m_condVar.notify_one();
    // lock is automatically released when lock goes out of scope.
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
  bool try_pop(T &result) {
    // could use std::optional
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_queue.empty())
      return false;
    result = m_queue.front();
    m_queue.pop_front();
    return true;
  }

  /**
   * @brief Waits until an element is available and then pops it.
   *
   * This function blocks until an element is available in the queue.
   * It uses a condition variable to wait, rechecking the condition upon
   * wake-up.
   *
   * @param result Reference to store the popped element.
   */
  void wait_and_pop(T &result) {
    std::unique_lock<std::mutex> lock(m_mutex);

    // Wait until the queue is not empty.
    m_condVar.wait(lock, [this] { return !m_queue.empty(); });
    result = m_queue.front();
    m_queue.pop_front();
  }

  /**
   * @brief Attempts to peek at the front element without popping.
   *
   * @param result Reference to store the front element if available.
   * @return true if the queue was non-empty and result was populated; false
   * otherwise.
   */
  bool front(T &result) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_queue.empty()) {
      return false;
    }
    result = m_queue.front(); // will copy/move
    return true;
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
    m_queue.clear();
  }

  /**
   * @brief Returns the number of elements dropped due to queue overflow.
   */
  size_t get_dropped_count() const noexcept { return dropped_count_.load(); }

  /**
   * @brief Returns the current size of the queue.
   */
  size_t size() const noexcept {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.size();
  }

private:
  boost::circular_buffer<T>
      m_queue; ///< Underlying circular queue for holding the data.
  mutable std::mutex m_mutex;        ///< Mutex to protect the queue.
  std::condition_variable m_condVar; ///< Condition variable for managing waits.
  size_t m_maxSize = 0; ///< Max size of the queue, 0 for unlimited size
  bool m_dropOldest =
      false; ///< If true, drop the oldest element when the queue is full
  std::atomic<size_t> dropped_count_{0}; ///< Count of dropped elements
};
