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
#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "algorithm.h"

typedef struct PID
{
  float prev_error;      // n-1 error
  float current_val; // Current value
  float integral;    // integral value
} PID;

// Gains            DEFAULTS
#define KP (0.420f)  // 0.5f
#define KI (0.0f)  // 0.1f
#define KD (0.0f) // 0.25f

float delta_duty(struct Result pnts);
void update_pid(PID* pid, float setpoint, float actpoint, float min, float max);

#endif
