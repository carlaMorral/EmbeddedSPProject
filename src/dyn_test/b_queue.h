/*
 * b_queue.h
 */

#ifndef B_QUEUE_H
#define B_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

typedef struct Queue {
    uint8_t data[32];
    uint8_t idx_pop;
    uint8_t idx_push;
    uint8_t data_pending;
    pthread_mutex_t mutex;
} Queue_t;

typedef enum _queue_ret {
    QUEUE_OK = 0,
    QUEUE_EMPTY = 1,
    QUEUE_FULL = 2,
    QUEUE_ERR = 3,
} QUEUE_RET;

// Both type definitions are equivalent for structures
struct Queue q_rx;
Queue_t q_tx;

void init_queue(Queue_t *q);

QUEUE_RET queue_push(uint8_t data, Queue_t *q);

QUEUE_RET queue_pop(uint8_t *data, Queue_t *q);

bool queue_is_empty(Queue_t *q);

bool queue_is_full(Queue_t *q);

#endif /* B_QUEUE_H */
