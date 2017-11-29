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
#include "PWM.h"
#include "camera.h"
#include "common.h"
#include "control.h"
#include "algorithm.h"

#ifdef DEBUG
#include "serial.h"
#endif

void initialize(void);


/* Motors:
 *   1: forwards
 *   0: backwards
 *   Rear:
 *     J9(4) 3.3v -> J10(3) H-Bridge Enable
 *   Right Rear:
 *     FTM0_CH0 - PTC1 (Dir 0, PWM) -> J1(7) H-Bridge A IN2
 *     FTM0_CH1 - PTC2 (Dir 1, PWM) -> J1(5) H-Bridge A IN1
 *   Left Rear:
 *     FTM0_CH2 - PTC3 (Dir 0, PWM) -> J10(10) H-Bridge B IN2
 *     FTM0_CH3 - PTC4 (Dir 1, PWM) -> J10(12) H-Bridge B IN1
 *   Servo:
 *     FTM3_CH0 - PTD0 (PWM) -> Servo (White)
 *
 * Camera:
 *   PTB9              - camera clk
 *   PTB23             - camera SI
 *   ADC0_DP3/ADC1_DP0 - camera out, ADC0 in
 *
 * Bluetooth (HC-06):
 *   PTA1(UART0_RX) - TXD
 *   PTA2(UART0_TX) - RXD
 *   3.3V           - Power 3.6V-6V (Hey, it works)
 */
int32_t main(void)
{
  uart_init();
    uint8_t str[RX_BUF_SZ];
  uart_put("AT+BAUD9600");
  uart_put("AT+VERSION\r");
  while(1)
  {
    //uart_put("WAITING\r\n");
    uart_get(str);
    uart_put(str);
    uart_putchar('\r');
  }
  
  uint16_t line[N_CAM_PNTS];
  struct Result pnts;
  float steer_duty = 7.5; //TODO
  PID servo_pid, dc_pid;
  
  initialize();

#ifdef DEBUG_CAM // TODO remove DDEBUG_CAM macro from compilation line for comp.
  //debug_camera();
#endif

  // Straight line
  update_pid(&dc_pid, 40, dc_pid.current_val, (float)MIN_DCMOT_DUTY, (float)MAX_DCMOT_DUTY);
  SetDCMotDuty(dc_pid.current_val);  // Slow pls TODO 30 MIN do
  
  // Camera
  for(;;) // ctrl+f all TODOs....
  {
    // Get camera line scan output
    get_line(line);
    // Detect line positions
    pnts = find_edges(line);
    
    update_pid(&dc_pid, 40, dc_pid.current_val, (float)MIN_DCMOT_DUTY, (float)MAX_DCMOT_DUTY);
    SetDCMotDuty(dc_pid.current_val);
    
    // Off track safety measure
    // TODO: move to state handling logic
    if (!pnts.l_pnt && !pnts.r_pnt)
    {
      GPIOB_PCOR = (1 << 22); // Red LED
      while(dc_pid.current_val)
      {
        update_pid(&dc_pid, 0, dc_pid.current_val, (float)MIN_DCMOT_DUTY, (float)MAX_DCMOT_DUTY); // TODO
        SetDCMotDuty(dc_pid.current_val);
      }
      break;
    }
    // TODO: no edges(intersection)? maybe pnts 0 & 127...or min/max distance between points
    
    // DC
    // TODO
    
    // Make control adjustments Change steering duty (TODO init servo_pid)
    if (servo_ready())
    {
      steer_duty = SERVO_SCALAR * ((float)(pnts.r_pnt + pnts.l_pnt) / 2.0f - N_CAM_PNTS + 1) * -1.0f + SERVO_BIAS;  // midpoint
      update_pid(&servo_pid, steer_duty, servo_pid.current_val, (float)MIN_SERVO_DUTY, (float)MAX_SERVO_DUTY);
      SetServoDuty(servo_pid.current_val);
    }

    // Debug printing
    DPRINT("left point: %u, right point: %u\r\n", pnts.l_pnt, pnts.r_pnt);
    DPRINT("steer_duty: %f\r\n", steer_duty);
    DDELAY; // Debug delay
  }
  
  return 0;
}


/* Initialize suitable modules
 */
void initialize(void)
{
#ifdef DEBUG
  // Initialize UART
  uart_init();
#endif

  // Initialize FTMs for PWM
  InitDCMotPWM();
  InitServoPWM();
  
  // 0 speed, 0 deg turn
  SetDCMotDuty(0);
  SetServoDuty(7.5); // TODO
  
  // Initialize camera
  init_camera();
}
