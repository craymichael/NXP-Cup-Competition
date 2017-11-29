/* A minimal FIFO queue implementation.
 * Author: Zach Carmichael
 */
#ifndef _QUEUE_H_
#define _QUEUE_H_

#include "MK64F12.h"

typedef struct ch_queue_t
{
    uint32_t size;
    uint32_t capacity;
    uint8_t* data;
    uint32_t i_front;
    uint32_t i_back;
} ch_queue_t;

void     ch_queue_init(ch_queue_t* queue, uint8_t* data, uint32_t capacity);
void     ch_queue_push(ch_queue_t* queue, uint8_t data);
uint8_t  ch_queue_pop(ch_queue_t* queue);
uint32_t ch_queue_full(ch_queue_t* queue);
uint32_t ch_queue_empty(ch_queue_t* queue);

#endif

