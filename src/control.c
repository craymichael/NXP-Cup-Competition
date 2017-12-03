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


void pid_init(PID* pid, float setpoint, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->setpoint = setpoint;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}


/* 
 * pid*:     Pointer to PID instance
 * setpoint: Desired thing (target)
 * actpoint: Actual thing
 */
float pid_compute(PID* pid, float actpoint)
{
  float error, derivative, current_val;
  
  // Compute error
  error = pid->setpoint - actpoint;
  
  // Update integral value
  pid->integral += error;
  
  // Compute derivative
  derivative = error - pid->prev_error;
  
  // Update output
  current_val = pid->kp * error +         // Proportion
                pid->ki * pid->integral + // Integral
                pid->kd * derivative;     // Derivative
  
  // Save error
  pid->prev_error = error;
  
  return current_val;
}
