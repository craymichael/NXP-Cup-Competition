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
#include <MK64F12.h>
#include "algorithm.h"
#include "common.h"

// Return max of A and B
#define MAX(A,B) (A < B ? B : A)


/* A very naive edge (line) finding algorithm
 *
 * Note: line gets modifed in this function
 */
struct Result find_edges(uint16_t* line)
{
  struct Result r = {0,N_CAM_PNTS-1};
  uint32_t max;
  
  // Smooth data (5-point average) & find max
  line[0] = (line[0] + line[1] + line[2]) / 3;
  max = line[0];
  line[1] = (line[0] + line[1] + line[2] + line[3]) / 4;
  max = MAX(line[0],max);
  for(uint32_t i = 2; i < N_CAM_PNTS - 2; ++i)
  {
    line[i] = (line[i-2] + line[i-1] + line[i] + line[i+1] + line[i+2]) / 5;
    max = MAX(line[i],max);
  }
  line[N_CAM_PNTS-2] = (line[N_CAM_PNTS-4] + line[N_CAM_PNTS-3] + line[N_CAM_PNTS-2] + line[N_CAM_PNTS-1]) / 4;
  max = MAX(line[N_CAM_PNTS-2],max);
  line[N_CAM_PNTS-1] = (line[N_CAM_PNTS-3] + line[N_CAM_PNTS-2] + line[N_CAM_PNTS-1]) / 3;
  max = MAX(line[N_CAM_PNTS-1],max);
  
  // Find left & right points in binarized data
  for(int32_t i = (N_CAM_PNTS/2)-1; i >= 0; --i)
  {
    // Check if value is interpreted as 0
    if((line[i] * CAM_THRESH) < max) {
      r.r_pnt = (uint32_t)i;
      break;
    }
  }
  for(uint32_t i = N_CAM_PNTS/2; i < N_CAM_PNTS; ++i)
  {
    // Check if value is interpreted as 0
    if((line[i] * CAM_THRESH) < max) {
      r.l_pnt = i;
      break;
    }
  }
  
  return r;
}
