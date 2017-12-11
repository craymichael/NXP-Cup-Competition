/* Copyright 2017 Carmichael, Lindberg
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* file: queue.c
 *
 * Implements a uint8_t (unsigned char) queue
 */
#include "queue.h"


// Initializes queue with parameters
void ch_queue_init(ch_queue_t* queue, uint8_t* data, uint32_t capacity)
{
  queue->capacity = capacity;
  queue->size = 0;
  queue->i_front = 0;
  queue->i_back = 0;
  queue->data = data;
}


// Is the queue full?
uint32_t ch_queue_full(ch_queue_t* queue)
{
  return queue->size == queue->capacity;
}


// Is the queue empty?
uint32_t ch_queue_empty(ch_queue_t* queue)
{
  return queue->size == 0;
}


// Push onto queue if not full
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


// Pop from queue if not empty
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
