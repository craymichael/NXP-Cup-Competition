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
#ifndef _PWM_H_
#define _PWM_H_

#define DEFAULT_SYSTEM_CLOCK (20485760u) /* Default System clock value */

// === DC Motor ===
// PWM frequency
#define PWM_DCMOT_FREQ (10000u)
// === Servo Motor ===
// PWM frequency
#define PWM_SERVO_FREQ (50u)
// Maximum and minimum duty
#define MIN_SERVO_DUTY (5u)
#define MAX_SERVO_DUTY (10u)

void SetDCMotDuty(uint32_t DutyCycle, uint32_t dir);
void SetServoAngle(uint32_t duty);
void InitPWM(void);
void InitServoPWM(void);

#endif /* PWM_H_ */
