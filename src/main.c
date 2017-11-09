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
#include "serial.h"
#include "PWM.h"
#include "camera.h"
#include "common.h"
#include "control.h"
#include "algorithm.h"

void initialize(void);


/* Motors:
 *   Left Rear:
 *     FTM0_CH0 - PTC1 (Dir 1, PWM) -> J10(12) H-Bridge B IN1
 *     FTM0_CH1 - PTC2 (Dir 0, PWM) -> J10(10) H-Bridge B IN2
 *   Right Rear:
 *     FTM0_CH2 - PTC3 (Dir 1, PWM) -> J1(5) H-Bridge A IN1
 *     FTM0_CH3 - PTC4 (Dir 0, PWM) -> J1(7) H-Bridge A IN2
 *   Servo:
 *     FTM3_CH0 - PTD0 (PWM) -> Servo (White)
 *
 * Camera:
 *   PTB9              - camera clk
 *   PTB23             - camera SI
 *   ADC0_DP3/ADC1_DP0 - camera out, ADC0 in
 */
int32_t main(void)
{
  uint16_t* line;
  struct Result pnts;
  float steer_delta, steer_duty = 7.5; //TODO
  
  initialize();

#ifdef DEBUG_CAM
  debug_camera();
#endif

  // Straight line
  SetDCMotDuty(20, 1);  // Slow pls
  
  // Camera
  for(;;)
  {
    // Get camera line scan output
    get_line(line);
    // Detect line positions
    pnts = find_edges(line);
    // Make control adjustments
    steer_delta = delta_duty(pnts);
    
    // Change steering duty
    steer_duty += steer_delta;
    SetServoDuty(steer_duty);
  }
  
  return 0;
}


/* Initialize suitable modules
 */
void initialize(void)
{
  // Initialize UART
  uart_init();

  // Initialize FTMs for PWM
  InitDCMotPWM();
  InitServoPWM();
  
  // 0 speed, 0 deg turn
  SetDCMotDuty(0, 1);
  SetServoDuty(7.5); // TODO
  
  // Initialize camera
  init_camera();
}