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
/* file: queue.h
 *
 * Implements a uint8_t (unsigned char) queue
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
