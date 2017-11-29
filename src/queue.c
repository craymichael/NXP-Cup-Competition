/* A minimal FIFO queue implementation.
 * Author: Zach Carmichael
 */
#include "queue.h"


void ch_queue_init(ch_queue_t* queue, uint8_t* data, uint32_t capacity)
{
  queue->capacity = capacity;
  queue->size = 0;
  queue->i_front = 0;
  queue->i_back = 0;
  queue->data = data;
}


uint32_t ch_queue_full(ch_queue_t* queue)
{
  return queue->size == queue->capacity;
}


uint32_t ch_queue_empty(ch_queue_t* queue)
{
  return queue->size == 0;
}


void ch_queue_push(ch_queue_t* queue, uint8_t data)
{
  if (queue->size == queue->capacity)
    return;

  // Increase size
  queue->size += 1;
  // Append data
  queue->data[queue->i_back] = data;
  // Update back index
  if (queue->i_back < queue->capacity-1)
    ++(queue->i_back);
  else
    queue->i_back = 0;
}


uint8_t ch_queue_pop(ch_queue_t* queue)
{
  uint8_t popped;

  if (queue->size == 0)
    return 0;

  // Decrement size
  queue->size -= 1;
  // Pop data
  popped = queue->data[queue->i_front];
  // Update front index
  if (queue->i_front < queue->capacity-1)
    ++(queue->i_front);
  else
    queue->i_front = 0;

  return popped;
}
