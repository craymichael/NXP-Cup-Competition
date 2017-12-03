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

// Variable speed
#include <math.h>
// Parsing BT cmds
#include <string.h>
#include <stdlib.h>

void initialize(void);
void run(float kp, float ki, float kd, float minspeed, float maxspeed);


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
  // PID parameters
  float kp = KP,
        ki = KI,
        kd = KD;
  // Min max DC motor speeds
  float minspeed = MINSPEED,
        maxspeed = MAXSPEED;
  
#ifdef DEBUG
  // Input command
  uint8_t cmd[50];
#endif
  
  // Init K64F modules
  initialize();
  
  for(;;)
  {
    DBG(
      // Wait until BT command received
      DPRINT("Enter Command:\r\n");
      uart_get(cmd);
      if(!strcmp((char*)cmd, CSTART))
      {
        // Run
        DPRINT("Running with:\r\n");
        DPRINT("  kp:       %f\r\n", kp);
        DPRINT("  ki:       %f\r\n", ki);
        DPRINT("  kd:       %f\r\n", kd);
        DPRINT("  minspeed: %f\r\n", minspeed);
        DPRINT("  maxspeed: %f\r\n", maxspeed);
        DPRINT("==========================\r\n\r\n");
        run(kp, ki, kd, minspeed, maxspeed);
        DPRINT("Stopping run.\r\n");
      } else if(!strcmp((char*)cmd, CPID))
      {
        // Get KP
        DPRINT("Enter KP:\r\n");
        uart_get(cmd);
        kp = strtof((char*)cmd, NULL);
        // Get KI
        DPRINT("Enter KI:\r\n");
        uart_get(cmd);
        ki = strtof((char*)cmd, NULL);
        // Get KD
        DPRINT("Enter KD:\r\n");
        uart_get(cmd);
        kd = strtof((char*)cmd, NULL);
      } else if(!strcmp((char*)cmd, CSPEEDS))
      {
        // Get KI
        DPRINT("Enter minspeed:\r\n");
        uart_get(cmd);
        minspeed = strtof((char*)cmd, NULL);
        // Get KD
        DPRINT("Enter maxspeed:\r\n");
        uart_get(cmd);
        maxspeed = strtof((char*)cmd, NULL);
      }
    );
    // Else use buttons...TODO
  }
  
  return 0;
}


void run(float kp, float ki, float kd, float minspeed, float maxspeed)
{
  uint16_t line[N_CAM_PNTS];
  struct Result pnts;
  float steer_duty,
        midpoint,
        error;
<<<<<<< .mine


=======
  uint32_t motor_duty;
  
>>>>>>> .theirs
  PID servo_pid;
  
#ifdef DEBUG
  // Input command
  uint8_t cmd[50];
#endif
  
  // 0 speed, 0 deg turn
  SetDCMotDuty(0);
  SetServoDuty(CTR_SERVO_DUTY);
  
#ifdef DUMB_WAY
  pid_init(&servo_pid, 0.0f, kp, ki, kd);
#else
  // Init Servo PID (0.0 is line center)
  pid_init(&servo_pid, 0.0f, kp, ki, kd);
#endif

  // Straight line, straight wheels
  SetDCMotDuty(minspeed);
  steer_duty = CTR_SERVO_DUTY;
  
  // Camera
  for(;;)
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
      SetDCMotDuty(minspeed);
      for(uint32_t i=0;i<1000;++i);
      SetDCMotDuty(0);
      return;
    }
    
    // DC Variable Speed (straight)
    SetDCMotDuty(maxspeed-(maxspeed-minspeed)*fabsf(CTR_SERVO_DUTY-steer_duty)/(MAX_SERVO_DUTY-CTR_SERVO_DUTY));
    
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
      CLIP(steer_duty, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
      stateSet(steer_duty,motor_duty);
      SetServoDuty(steer_duty);
#else
      // Normalize midpoint to error bounds [-2.5,+2.5] for servo
      error = NORM_CAM2SERVO(midpoint-CAM_MID_PNT);
      steer_duty += pid_compute(&servo_pid, error);
      CLIP(steer_duty, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
      stateSet(steer_duty,motor_duty);
      SetServoDuty(steer_duty);
#endif
    }
    
    // Check if command received to stop running
    DBG(
      if(uart_hasdata())
      {
        uart_get(cmd);
        if(!strcmp((char*)cmd, CSTOP))
          return;
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
  
  // Initialize camera
  init_camera();
  
#ifdef DEBUG_CAM
  debug_camera();
#endif
}
