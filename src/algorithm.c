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


/* An edge (line) finding algorithm
 * Worst-case complexity O(n^2)
 *
 * Note: line gets modifed in this function
 */
struct Result find_edges(uint16_t* line, uint32_t midpoint_p)
{
  struct Result r = {0,N_CAM_PNTS-1};
  uint32_t max;
  float thresh;
  
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
  
  // Detect if car may be off the track
  /*if (max < NOTRACK_THRESH)
  {
    r.l_pnt = 0;
    r.r_pnt = 0;
    return r;
  }*/
  
  // Set theshold using max
  thresh = (float)max * CAM_THRESH;
  
  // Find left & right points in "binarized" data
  if (line[midpoint_p-1] < thresh || line[midpoint_p] < thresh) // if either midpoint is 0
  {
    for(int32_t i=0; i<(midpoint_p>(N_CAM_PNTS-midpoint_p) ? midpoint_p : (N_CAM_PNTS-midpoint_p-1)); ++i)
    {
      // Check if value is interpreted as 1 (towards left)
      if((midpoint_p+i) < N_CAM_PNTS && line[midpoint_p+i] >= thresh) {
        r.r_pnt = midpoint_p+i-1;
        // Find left point
        for(uint32_t j=midpoint_p+i+1; j<N_CAM_PNTS; ++j)
        {
          // Check if value is interpreted as 0
          if(line[j] < thresh) {
            r.l_pnt = j;
            break;
          }
        }
        break;
      }
      // Check if value is interpreted as 1 (towards right)
      if(((int32_t)midpoint_p-i-1) >= 0 && line[midpoint_p-i-1] >= thresh) {
        r.l_pnt = midpoint_p-i;
        // Find right point
        for(int32_t j=midpoint_p-i-2; j>=0; --j)
        {
          // Check if value is interpreted as 0
          if(line[j] < thresh) {
            r.r_pnt = (uint32_t)j;
            break;
          }
        }
        break;
      }
    }
  } else 
  {
    // Find points moving out from center
    for(int32_t i=midpoint_p-1; i >= 0; --i)
    {
      // Check if value is interpreted as 0
      if(line[i] < thresh) {
        r.r_pnt = (uint32_t)i;
        break;
      }
    }
    for(uint32_t i=midpoint_p; i < N_CAM_PNTS; ++i)
    {
      // Check if value is interpreted as 0
      if(line[i] < thresh) {
        r.l_pnt = i;
        break;
      }
    }
  }
  
  return r;
}
