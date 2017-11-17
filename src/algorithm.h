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
#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "MK64F12.h"

// Threshold (percentage of max value of line that is interpreted as 0 or black)
#define CAM_THRESH     (0.85f)
// No track threshold
#define NOTRACK_THRESH (10000u)

struct Result {
  uint32_t l_pnt;
  uint32_t r_pnt;
};

struct Result find_edges(uint16_t* line);

#endif
