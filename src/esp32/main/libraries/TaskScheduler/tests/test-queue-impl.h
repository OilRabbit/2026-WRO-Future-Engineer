// test-queue-impl.h - Header for Linux queue implementation
// Declares helper functions for queue management in tests

#ifndef TEST_QUEUE_IMPL_H
#define TEST_QUEUE_IMPL_H

#include <cstddef>

// #define _TASK_HEADER_AND_CPP
// #define _TASK_THREAD_SAFE
// #define _TASK_STATUS_REQUEST
// #define _TASK_TIMEOUT_TASK_THREAD_SAFE

#include "TaskSchedulerDeclarations.h"

// Helper functions for testing
void clearTaskRequestQueue();
size_t getQueueSize();

// Maximum queue size constant
#define MAX_QUEUE_SIZE 100

bool _task_dequeue_request(_task_request_t* req);
bool _task_enqueue_request(_task_request_t* req);

#endif // TEST_QUEUE_IMPL_H
