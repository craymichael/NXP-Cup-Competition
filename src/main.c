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
#include "states.h"

#ifdef DEBUG
// For Bluetooth/Serial communications
#include "serial.h"
#endif

#include <math.h>  // TODTODOTODODODO

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
  uint16_t line[N_CAM_PNTS];
  struct Result pnts;
  float steer_duty,
        midpoint,
        error;
  uint32_t motor_duty;
  
  PID servo_pid;
  
  // Init K64F modules
  initialize();
  
#ifdef DUMB_WAY
  pid_init(&servo_pid, 0.0f, KP, KI, KD);
#else
  // Init Servo PID (0.0 is line center)
  pid_init(&servo_pid, 0.0f, KP, KI, KD);
#endif

  // Straight line, straight wheels
  SetDCMotDuty(40);
  steer_duty = CTR_SERVO_DUTY;
  
  // Camera
  for(;;) // ctrl+f all TODOs....
  {
    // Get camera line scan output
    get_line(line);
    // Detect line positions
    pnts = find_edges(line);
    
    // Off track safety measure
    // TODO: move to state handling logic
    if (!pnts.l_pnt && !pnts.r_pnt)
    {
      GPIOB_PCOR = (1 << 22); // Red LED (TODO no debug cam to init RED)
      /*while(dc_pid.current_val)
      {
        update_pid(&dc_pid, 0, dc_pid.current_val, (float)MIN_DCMOT_DUTY, (float)MAX_DCMOT_DUTY); // TODO
        SetDCMotDuty(dc_pid.current_val);
      }*/
      SetDCMotDuty(0);
      stateSet(0.0f,0);
      break;
    }
    // TODO: no edges(intersection)? maybe pnts 0 & 127...or min/max distance between points
    
    // DC Variable Speed (straight)
    motor_duty = 60.0f - (60.0f-40.0f)*fabsf(7.5f-steer_duty)/2.5f;
    SetDCMotDuty(motor_duty);
    
    // Make control adjustments Change steering duty (TODO init servo_pid)
    if (servo_ready())
    {
      // Compute camera midpoint
      midpoint = (float)(pnts.r_pnt + pnts.l_pnt) / 2.0f;
      
#ifdef DUMB_WAY 
      // Normalize midpoint to bounds [5,10] for servo
      error = SERVO_SCALAR * (midpoint - N_CAM_PNTS + 1) * -1.0f + SERVO_BIAS;
      // Double relative error
      error = error + (error - CTR_SERVO_DUTY) * 2.0f;
      /*
      // Squared relative error
      float sqerr = (steer_duty - CTR_SERVO_DUTY);
      if(sqerr < 0)
        steer_duty = steer_duty - sqerr * sqerr;
      else
        steer_duty = steer_duty + sqerr * sqerr;
      */
      
      steer_duty = pid_compute(&servo_pid, steer_duty-error);
      CLIP(steer_duty, (float)MIN_SERVO_DUTY, (float)MAX_SERVO_DUTY);
      stateSet(steer_duty,motor_duty);
      SetServoDuty(steer_duty);
#else
      // Normalize midpoint to error bounds [-2.5,+2.5] for servo
      error = NORM_CAM2SERVO(midpoint-CAM_MID_PNT);
      steer_duty += pid_compute(&servo_pid, error);
      CLIP(steer_duty, (float)MIN_SERVO_DUTY, (float)MAX_SERVO_DUTY);
      stateSet(steer_duty,motor_duty);
      SetServoDuty(steer_duty);
#endif
    }
    
    //
    DBG(
      if(uart_hasdata())
      {
        
      }
    );

    // Debug printing
    DDELAY(750,
      DPRINT("left point: %u, right point: %u\r\n", pnts.l_pnt, pnts.r_pnt);
      DPRINT("midpoint: %f\r\n", midpoint);
      DPRINT("steer_duty: %f\r\n", steer_duty);
      DPRINT("error: %f\r\n", error);
      DPRINT("\r\n");
    );
  }
  
  while(1);
  
  return 0;
}


/* Initialize suitable modules
 */
void initialize(void)
{
  DBG(
    // Initialize UART
    uart_init();
  );

  // Initialize FTMs for PWM
  InitDCMotPWM();
  InitServoPWM();
  
  // 0 speed, 0 deg turn
  SetDCMotDuty(0);
  SetServoDuty(CTR_SERVO_DUTY);
  
  // Initialize camera
  init_camera();
  
#ifdef DEBUG_CAM
  debug_camera();
#endif
}
