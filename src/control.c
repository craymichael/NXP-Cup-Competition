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
#include "algorithm.h"
#include "control.h"
#include "common.h"


// A simple control algorithm
//   avg centered distance about 20?
float delta_duty(struct Result pnts)
{
  int32_t ldist = pnts.l_pnt - N_CAM_PNTS/2,
          rdist = N_CAM_PNTS/2 - pnts.r_pnt;
  // If the distance to the right point is further than to the left...
  if(rdist > ldist) {
    return 0.005f;
  } else {
    return -0.005f;
  }
}
