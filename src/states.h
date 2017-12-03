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
#ifndef _STATES_H_
#define _STATES_H_
/*
GPIOB_PCOR = (1UL << 21); Turn on Blue LED
GPIOB_PCOR = (1UL << 22); Turn on Red LED
GPIOE_PCOR = (1UL << 26); Turn on Green LED

GPIOB_PSOR = (1UL << 21); Turn off Blue LED
GPIOB_PSOR = (1UL << 22); Turn off Red LED
GPIOE_PSOR = (1UL << 26); Turn off Green LED
*/
#define OFFTRACK                GPIOB_PCOR |= (1UL << 22); 
#define LEFT                    GPIOB_PCOR |= (1UL << 21) | (1UL << 22);  
#define RIGHT                   GPIOE_PCOR |= (1UL << 26); GPIOB_PCOR |= (1UL << 21);
#define STRAIGHT                GPIOE_PCOR |= (1UL << 26);
#define FINISHED
#define OFFLED                  GPIOB_PSOR |= (1UL << 21) | (1UL << 22); GPIOE_PSOR |= (1UL << 26);
 void stateSet(float steerDuty, int32_t Motorduty);
 void LED_Init(void);

#endif



    
