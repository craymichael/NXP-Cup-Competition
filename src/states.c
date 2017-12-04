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
#include "MK64F12.h"
#include "common.h"
#include "control.h"
#include "algorithm.h"
#include "states.h"

#ifdef DEBUG
// For Bluetooth/Serial communications
#include "serial.h"
#endif

#include <math.h>  // TODTODOTODODODO

void stateSet(float steerDuty, int32_t motorDuty){
    if(motorDuty == 0)
    {
        OFFLED
        OFFTRACK
    }
    if(steerDuty < 7.0f)
    {
        OFFLED
        LEFT
    }
    if(steerDuty > 8.0f)
    {
        OFFLED
        RIGHT
    }
    else
    {
        OFFLED
        STRAIGHT   
    }
}

 void LED_Init(void){
	// Enable clocks on Ports B and E for LED timing
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	// Configure the Signal Multiplexer for GPIO
	PORTB_PCR22 = PORT_PCR_MUX(1);
	PORTB_PCR21 = PORT_PCR_MUX(1);
	PORTE_PCR26 = PORT_PCR_MUX(1);
	
	// Switch the GPIO pins to output mode
	GPIOB_PDDR |= (1<<21);//BlueLED 
	GPIOB_PDDR |= (1<<22);//RedLED
	GPIOE_PDDR |= (1<<26);//GreenLED
	// Turn off the LEDs
	GPIOB_PDOR |= (1<<21);
	GPIOB_PDOR |= (1<<22);
	GPIOE_PDOR |= (1<<26);
}
