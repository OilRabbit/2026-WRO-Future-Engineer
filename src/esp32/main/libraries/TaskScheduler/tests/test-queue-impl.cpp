// test-queue-impl.cpp - Linux queue implementation for thread-safe testing
// This file provides the actual implementation of the queue functions
// to override the weak symbols defined in TaskScheduler.h

#include "Arduino.h"

// Define thread-safe features before including TaskScheduler
// defined in the test.yml

// #define _TASK_HEADER_AND_CPP
// #define _TASK_THREAD_SAFE
// #define _TASK_STATUS_REQUEST
// #define _TASK_TIMEOUT_TASK_THREAD_SAFE

#include "test-queue-impl.h"
#include "TaskSchedulerDeclarations.h"

#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

// ============================================
// Thread-Safe Queue Implementation for Linux
// ============================================

// Queue for task requests
std::queue<_task_request_t> taskRequestQueue;
std::mutex queueMutex;
std::condition_variable queueCV;


/**
 * @brief Enqueue a task request (Linux implementation)
 *
 * Thread-safe enqueue operation using std::mutex.
 * Simulates FreeRTOS queue behavior for Linux testing.
 *
 * @param req Pointer to the request structure
 * @return true if enqueued successfully, false if queue is full
 */
bool _task_enqueue_request(_task_request_t* req) {
    std::unique_lock<std::mutex> lock(queueMutex);

    // Check queue size limit
    if (taskRequestQueue.size() >= MAX_QUEUE_SIZE) {
        return false;  // Queue full
    }

    // Add request to queue
    taskRequestQueue.push(*req);

    // Notify waiting threads
    queueCV.notify_one();

    return true;
}

/**
 * @brief Dequeue a task request (Linux implementation)
 *
 * Thread-safe dequeue operation. Non-blocking for scheduler.
 *
 * @param req Pointer to store the dequeued request
 * @return true if request dequeued, false if queue empty
 */
bool _task_dequeue_request(_task_request_t* req) {
    std::unique_lock<std::mutex> lock(queueMutex);

    if (taskRequestQueue.empty()) {
        return false;  // No requests
    }

    // Get request from queue
    *req = taskRequestQueue.front();
    taskRequestQueue.pop();

    return true;
}

/**
 * @brief Helper to clear the queue between tests
 */
void clearTaskRequestQueue() {
    std::unique_lock<std::mutex> lock(queueMutex);
    while (!taskRequestQueue.empty()) {
        taskRequestQueue.pop();
    }
}

/**
 * @brief Helper to get current queue size
 */
size_t getQueueSize() {
    std::unique_lock<std::mutex> lock(queueMutex);
    return taskRequestQueue.size();
}
