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

#define DEFAULT_SYSTEM_CLOCK (20485760u) // System clock value
#define BAUD_RATE            (9600u)     // UART Baud Rate

void initialize(void);


/* Motors:
 *   J1 right wheel 5,7
 *   J10 left wheel 10,12
 * Camera:
 *   PTB9              - camera clk
 *   PTB23             - camera SI
 *   ADC0_DP3/ADC1_DP0 - camera out, ADC0 in
 */
int32_t main(void)
{
  initialize();

#ifdef DEBUG_CAM
  debug_camera();
#endif
  
  return 0;
}


/* Initialize suitable modules
 */
void initialize(void)
{
  // Initialize UART
  uart_init();

  // Initialize FTMs for PWM
  //InitPWM();
  //InitServoPWM();
  
  // Initialize camera
  init_camera();
}
