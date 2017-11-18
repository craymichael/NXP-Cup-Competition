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
#include "pwm.h"
#include "common.h"

#define FTM0_MOD_VALUE (DEFAULT_SYSTEM_CLOCK / PWM_DCMOT_FREQ - 1u)
#define FTM3_MOD_VALUE (DEFAULT_SYSTEM_CLOCK / (PWM_SERVO_FREQ * 128u) - 1u)

// Whether servo can steer
uint32_t can_steer = 0;


/*
 * Change the Motor Duty Cycle and Frequency
 *   @param duty: (0 to 100%)
 *   @param dir:       1 for C4 active, else C3 active 
 */
void SetDCMotDuty(uint32_t duty, uint32_t dir)
{
  // Calculate the new cutoff value
  uint16_t mod =  (FTM0_MOD_VALUE * duty) / 100u,
           modc = (FTM0_MOD_VALUE * (duty + PWM_DCMOT_CORRECT)) / 100u;
  

  // Set outputs
  if(dir == 1) {
    // Rear wheel 1
    FTM0_C3V = mod;  // PTC4 (dir 1)
    FTM0_C2V = 0;
    // Rear wheel 2
    FTM0_C1V = modc; // PTC2 (dir 1)
    FTM0_C0V = 0;
  } else {
    // Rear wheel 1
    FTM0_C2V = mod;  // PTC3 (dir 0)
    FTM0_C3V = 0;
    // Rear wheel 2
    FTM0_C0V = modc; // PTC1 (dir 0)
    FTM0_C1V = 0;
  }
}


// Duty has to be between 5 and 10%
void SetServoDuty(float duty)
{
  uint16_t mod;
  
  // Negate boolean
  can_steer = 0;
  
  // Safety measure
  if(duty < MIN_SERVO_DUTY || duty > MAX_SERVO_DUTY)
    return;
  
  // Update mod value w/ duty amount
  mod = (uint16_t)(FTM3_MOD_VALUE * duty) / 100u;
  FTM3_C0V = mod;
}


void FTM3_IRQHandler(void)
{
  // Clear flag
  FTM3_SC &= ~FTM_SC_TOF_MASK;
  
  can_steer = 1;
}


uint32_t servo_ready(void)
{
  return can_steer;
}


/*
 * Initialize the FlexTimer for PWM
 */
void InitDCMotPWM(void)
{
  // Enable the Clock to the FTM0 Module
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

  // Enable clock on PORTs A, B, C
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK |
               SIM_SCGC5_PORTB_MASK |
               SIM_SCGC5_PORTC_MASK;

  // Route the output of FTM channel 0 to the pins
  // Use drive strength enable flag to high drive strength
  PORTC_PCR3 = PORT_PCR_MUX(0x4) |  // Ch2
               PORT_PCR_DSE_MASK;
  PORTC_PCR4 = PORT_PCR_MUX(0x4) |  // Ch3
               PORT_PCR_DSE_MASK;
  
  PORTC_PCR1 = PORT_PCR_MUX(0x4) |  // Ch0
               PORT_PCR_DSE_MASK;
  PORTC_PCR2 = PORT_PCR_MUX(0x4) |  // Ch1
               PORT_PCR_DSE_MASK;

  // 39.3.10 Disable Write Protection
  FTM0_MODE |= FTM_MODE_WPDIS_MASK;

  // 39.3.4 FTM Counter Value
  // Initialize the CNT to 0 before writing to MOD
  FTM0_CNT = 0;

  // 39.3.8 Set the Counter Initial Value to 0
  FTM0_CNTIN = 0;

  // 39.3.5 Set the Modulo resister
  FTM0_MOD = FTM0_MOD_VALUE;

  // 39.3.6 Set the Status and Control of both channels
  // Used to configure mode, edge and level selection
  // See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
  FTM0_C3SC |= FTM_CnSC_MSB_MASK |
               FTM_CnSC_ELSB_MASK;
  FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;

  // See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
  FTM0_C2SC |= FTM_CnSC_MSB_MASK |
               FTM_CnSC_ELSB_MASK;
  FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;
  
  // See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
  FTM0_C1SC |= FTM_CnSC_MSB_MASK |
               FTM_CnSC_ELSB_MASK;
  FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;

  // See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
  FTM0_C0SC |= FTM_CnSC_MSB_MASK |
               FTM_CnSC_ELSB_MASK;
  FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;

  // 39.3.3 FTM Setup
  // Set prescale value to 1 
  // Chose system clock source
  // Timer Overflow Interrupt Enable
  FTM0_SC = FTM_SC_PS(0x0) |
            FTM_SC_CLKS(0x1);
}


/* Initialize FTM3 for servo PWM control
 */
void InitServoPWM(void)
{
  // Enable FTM1 clock
  SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;

  // Enable PORTB clock
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

  // PTD0 (FTM3_CH0) mux
  PORTD_PCR0 = PORT_PCR_MUX(0x4) |
               PORT_PCR_DSE_MASK;

  // 39.3.10 Disable Write Protection
  FTM3_MODE |= FTM_MODE_WPDIS_MASK;

  // 39.3.4 FTM Counter Value
  // Initialize the CNT to 0 before writing to MOD
  FTM3_CNT = 0;

  // 39.3.8 Set the Counter Initial Value to 0
  FTM3_CNTIN = 0;

  // 39.3.5 Set the Modulo resister
  FTM3_MOD = FTM3_MOD_VALUE;

  // 39.3.6 Set the Status and Control of both channels
  // Used to configure mode, edge and level selection
  // See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
  FTM3_C0SC |= FTM_CnSC_MSB_MASK  |
               FTM_CnSC_ELSB_MASK;
  FTM3_C0SC &= ~FTM_CnSC_ELSA_MASK;

  // 39.3.3 FTM Setup
  // Set prescale value to 1 
  // Chose system clock source
  // Timer Overflow Interrupt Enable
  FTM3_SC = FTM_SC_PS(0x7)   |
            FTM_SC_CLKS(0x1) |
            FTM_SC_TOIE_MASK;
  // NVIC pls
  NVIC_EnableIRQ(FTM3_IRQn);
}
