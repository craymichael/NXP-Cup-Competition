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
#include "camera.h"
#include "common.h"

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
// new_line: whether new line is available from camera
int32_t pixcnt, new_line; // TODO(code style)
// line stores the current array of camera data, line_p the prior data
uint16_t line[N_CAM_PNTS];  // TODO(code style)
// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;    // TODO(code style)


#ifdef DEBUG_CAM // Camera Debugging function
  #include "serial.h"
  #include <stdio.h>

  void debug_camera(void)
  {
    uint8_t str[100];
    
    for(;;)
    {
      // Every 2 seconds
      if (new_line) {
        GPIOB_PCOR |= (1 << 22);

        // send the array over uart
        sprintf((char*)str,"%i\n\r",-1); // start value
        uart_put(str);

        for(uint32_t i = 0; i < 127; i++) {
          sprintf((char*)str,"%i\n", line[i]);
          uart_put(str);
        }

        sprintf((char*)str,"%i\n\r",-2); // end value
        uart_put(str);
        GPIOB_PSOR |= (1 << 22);
      }
    }
  }
#endif


/* Initialize camera-related modules */
void init_camera(void)
{
  pixcnt = -2;
  new_line = 0;
  
  init_GPIO();
  init_FTM2();
  init_ADC0();
  init_PIT();
}


void get_line(uint16_t* line_p)
{
  while(!new_line); // wait until new line available
  
  // Make sure line doesn't get messed with
  NVIC_DisableIRQ(FTM2_IRQn);
  
  for(uint32_t i = 0; i < N_CAM_PNTS; ++i) { // copy line
    line_p[i] = line[i];
  }
  
  // Resume camera captures
  NVIC_EnableIRQ(FTM2_IRQn);
}


/* ADC0 Conversion Complete */
void ADC0_IRQHandler(void)
{
  // Reading ADC0_RA clears the conversion complete flag
  ADC0VAL = ADC0_RA;
}


/* 
 * FTM2 handles the camera driving logic
 *    This ISR gets called once every integration period
 *        by the periodic interrupt timer 0 (PIT0)
 *    When it is triggered it gives the SI pulse,
 *        toggles clk for 128 cycles, and stores the line
 *        data from the ADC into the line variable
 */
void FTM2_IRQHandler(void)
{
  // Clear interrupt
  FTM2_SC &= ~FTM_SC_TOF_MASK;
  
  // Toggle clk
  GPIOB_PTOR |= (1 << 9);
  
  new_line = 0; // new line unavailable

  // Line capture logic
  if (pixcnt < (N_CAM_PNTS * 2)) {
    switch(pixcnt) {
      case -1: // pixcnt < 2
        GPIOB_PSOR |= (1 << 23); // SI = 1
        break;
      case 1:  // pixcnt < 2
        GPIOB_PCOR |= (1 << 23); // SI = 0
        // ADC read
        line[0] = ADC0VAL;
        break;
      default: // 2 <= pixcnt < 256
        if (!GPIOB_PDOR) { // check for falling edge
          line[pixcnt/2] = ADC0VAL; // ADC read
        }
        break;
    }
    pixcnt += 1;
  } else {
    pixcnt = -2;  // reset counter
    new_line = 1; // new line available
    // Disable FTM2 interrupts (until PIT0 overflows and triggers another line capture)
    FTM2_SC &= ~FTM_SC_TOIE_MASK;
  }
}


/* PIT0 determines the integration period
 * When it overflows, it triggers the clock logic from
 * FTM2. Note the requirement to set the MOD register
 * to reset the FTM counter because the FTM counter is 
 * always counting, I am just enabling/disabling FTM2 
 * interrupts to control when the line capture occurs
 */
void PIT0_IRQHandler(void)
{
  // Clear interrupt
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

  // Setting mod resets the FTM counter
  FTM2_MOD = DEFAULT_SYSTEM_CLOCK / 100000;

  // Enable FTM2 interrupts (camera)
  FTM2_SC |= FTM_SC_TOIE_MASK;
}


/* Initialization of FTM2 for camera */
void init_FTM2(void)
{
  // Enable module clock
  SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

  // Func Func mode (for debugging)
  FTM2_CONF |= FTM_CONF_BDMMODE(0x3);

  // Disable Write Protection
  FTM2_MODE |= FTM_MODE_WPDIS_MASK | // Write 1 to WPDIS
               FTM_MODE_FTMEN_MASK;  // Enable all registers

  // Set the period (~10us)
  FTM2_MOD = DEFAULT_SYSTEM_CLOCK / 100000u - 1u;

  // Set edge-aligned mode
  FTM2_C0SC |= FTM_CnSC_MSB_MASK;

  // Enable High-true pulses
  // ELSB = 1, ELSA = 0
  FTM2_C0SC |= FTM_CnSC_ELSB_MASK;

  // Set output to '1' on init
  FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;

  // Initialize the CNT to 0 before writing to MOD
  FTM2_CNT = 0x0;

  // Set the Counter Initial Value to 0
  FTM2_CNTIN = 0x0;

  // 50% duty
  // EPWM period = (MOD-CNTIN + 0x0001) 
  // pulse width duty = (CnV - CNTIN )
  FTM2_C0V = DEFAULT_SYSTEM_CLOCK / 200000u - 1u;

  // Don't enable interrupts yet (disable)
  FTM2_SC &= ~FTM_SC_TOIE_MASK;

  // No prescalar, system clockm, and Set up interrupt
  FTM2_SC |= FTM_SC_CLKS(0x1) |
             FTM_SC_TOIE_MASK;

  // Actually init using OUTINIT values
  FTM2_MODE |= FTM_MODE_INIT_MASK;

  // Enable hardware trigger for channel 0
  FTM2_EXTTRIG |= FTM_EXTTRIG_CH0TRIG_MASK;

  // Enable IRQ
  NVIC_EnableIRQ(FTM2_IRQn);
}


/* Initialization of PIT timer to control 
 * integration period
 */
void init_PIT(void)
{
  // Setup periodic interrupt timer (PIT)
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

  // Enable clock for timers
  PIT_MCR &= ~PIT_MCR_MDIS_MASK;

  // Enable timers to continue in debug mode
  PIT_MCR &= ~PIT_MCR_FRZ_MASK;

  // PIT clock frequency is the system clock
  // Load the value that the timer will count down from
  PIT_LDVAL0 = INTEGRATION_TIME * DEFAULT_SYSTEM_CLOCK - 1u;

  // Enable timer interrupts
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;  

  // Enable the timer
  PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

  // Clear interrupt flag
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

  // Enable PIT IRQ
  NVIC_EnableIRQ(PIT0_IRQn);
}


/* Set up pins for GPIO
 *     PTB9         - camera clk
 *     PTB23        - camera SI
 *     PTB22        - red LED
 */
void init_GPIO(void)
{
  // Enable LED and GPIO so we can see results
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

  PORTB_PCR22 |= PORT_PCR_MUX(0x1); // Mux Red
  GPIOB_PDDR |= (1 << 22); // Red output
  GPIOB_PDOR |= (1 << 22); // Red off

  PORTB_PCR9 |= PORT_PCR_MUX(0x1); // Mux clk
  GPIOB_PDDR |= (1 << 9); // Clk output

  PORTB_PCR23 |= PORT_PCR_MUX(0x1); // Mux SI
  GPIOB_PDDR |= (1 << 23); // SI output
}


/* Set up ADC for capturing camera data */
void init_ADC0(void) {
  uint32_t calib;

  // Turn on ADC0
  SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;  

  // Single ended 16 bit conversion, no clock divider
  ADC0_CFG1 |= ADC_CFG1_MODE(0x3);

  // Do ADC Calibration for Singled Ended ADC. Do not touch.
  ADC0_SC3 = ADC_SC3_CAL_MASK;

  while ((ADC0_SC3 & ADC_SC3_CAL_MASK) != 0);

  calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
  calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
  calib = calib >> 1; calib |= 0x8000;
  ADC0_PG = calib;

  // Select hardware trigger.
  ADC0_SC2 |= ADC_SC2_ADTRG_MASK;

  // Set to single ended mode    
  ADC0_SC1A = ADC_SC1_ADCH(0x3) |  // DADP3
              ADC_SC1_AIEN_MASK;   // Interrupts on COCO

  // Set up FTM2 trigger on ADC0
  SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0xA)  | // FTM2 select
               SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
  SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK; // Pretrigger A
  
  // Enable NVIC interrupt
  NVIC_EnableIRQ(ADC0_IRQn);
}
