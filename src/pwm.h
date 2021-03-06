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
/* file: pwm.h
 *
 * Responsible for initializing FTM modules in a manner to control
 * H-bridges in multiple directions
 */
#ifndef _PWM_H_
#define _PWM_H_

// === DC Motor ===
// PWM frequency
#define PWM_DCMOT_FREQ    (10000u)
// Minimum and maximum duty
#define MAX_DCMOT_DUTY    (100.0f)
#define MIN_DCMOT_DUTY    (-100.0f)
// === Servo Motor ===
// PWM frequency
#define PWM_SERVO_FREQ    (50u)
// PWM correction duty
#define PWM_SERVO_CORRECT (0.0f)
// Maximum and minimum duty
#define MIN_SERVO_DUTY    (5.0f + PWM_SERVO_CORRECT)
#define MAX_SERVO_DUTY    (10.0f + PWM_SERVO_CORRECT)
// Center servo duty
#define CTR_SERVO_DUTY    ((MAX_SERVO_DUTY+MIN_SERVO_DUTY)/2.0f)

void SetDCMotDuty(float lduty, float rduty);
void SetServoDuty(float duty);

uint32_t servo_ready(void);

void InitDCMotPWM(void);
void InitServoPWM(void);

#endif /* PWM_H_ */
