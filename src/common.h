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
#ifndef _COMMON_H_
#define _COMMON_H_

/* Default System clock value */
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK (20485760u)

// Number of data points the camera outputs
#define N_CAM_PNTS           (128)

// "FUNCTIONS"
// Return max of A and B
#define MAX(A,B)  (A < B ? B : A)
// Return min of A and B
#define MIN(A,B)  (A < B ? A : B)
// Clip the value
#define CLIP(V,MINV,MAXV) { \
  if(V < MINV)              \
    V = MINV;               \
  else if (V > MAXV)        \
    V = MAXV;               \
}
// PWM Conversion factors
#include "pwm.h" // TODO move all parameters to this file?
// Servo:
// Conversion => CAM_OUT * SERVO SCALAR + SERVO_BIAS
#define SERVO_SCALAR ((float)MIN_SERVO_DUTY / (float)(N_CAM_PNTS-1))
#define SERVO_BIAS   ((float)MIN_SERVO_DUTY)

// Debug Stuff
#ifdef DEBUG
// These libraries become necessary
#include <stdio.h>
#include "serial.h" // TODO bluetooth?
// Debug printing
#define DPRINT(...) {                   \
  uint8_t _d_str_[128];                 \
  sprintf((char*)_d_str_, __VA_ARGS__); \
  uart_put(_d_str_);                    \
}
// Debug delay for somewhat arbitrary time
#define DDELAY {for(int i=0; i<5000000;++i);}
#else
// Empty macros
#define DPRINT(...)
#define DDELAY
#endif

#endif
