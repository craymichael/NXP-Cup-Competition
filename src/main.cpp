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

#define DEFAULT_SYSTEM_CLOCK (20485760u) /* Default System clock value */


// J1 right wheel 5,7
// J10 left wheel 10,12
int32_t main(void)
{
  
}


/* Initialize suitable modules
 */
void initialize()
{
  // Initialize UART
  uart_init();

  // Initialize FTMs for PWM
  InitPWM();
  InitServoPWM();
}
