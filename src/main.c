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

#include <float.h>
// States
#define CONTINUOUS   (0)
#define BRAKE        (1)
#define SPEEDUP      (2)
#define WAIT         (3)

#define BRAKE_FRAMES (10)
#define BRAKE_PWM    (-100.0f)

#define SPEEDUP_FRAMES (4)

#define PWM_DANGER   (30.0f)

void initialize(void);
void run(float kp, float ki, float kd, float minspeed, float maxspeed, float brakeframes, float brakepwm);

#define DUMB_WAY


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
  float brakeframes = BRAKE_FRAMES,
        brakepwm = BRAKE_PWM;
  
#ifdef DEBUG
  // Input command
  uint8_t cmd[50];
#endif
  
  // Init K64F modules
  initialize();
  
#ifdef DEBUG_CAM
  debug_camera();
#endif
  
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
        run(kp, ki, kd, minspeed, maxspeed, brakeframes, brakepwm);
        DPRINT("Stopping run.\r\n");
      } else if(!strcmp((char*)cmd, CPID))
      {
        // Get KP
        DPRINT("Enter KP(%f):\r\n", kp);
        uart_get(cmd);
        kp = strtof((char*)cmd, NULL);
        // Get KI
        DPRINT("Enter KI(%f):\r\n", ki);
        uart_get(cmd);
        ki = strtof((char*)cmd, NULL);
        // Get KD
        DPRINT("Enter KD(%f):\r\n", kd);
        uart_get(cmd);
        kd = strtof((char*)cmd, NULL);
      } else if(!strcmp((char*)cmd, CSPEEDS))
      {
        // Get minspeed
        DPRINT("Enter minspeed(%f):\r\n", minspeed);
        uart_get(cmd);
        minspeed = strtof((char*)cmd, NULL);
        // Get maxspeed
        DPRINT("Enter maxspeed(%f):\r\n", maxspeed);
        uart_get(cmd);
        maxspeed = strtof((char*)cmd, NULL);
      } else if(!strcmp((char*)cmd, CBRAKE))
      {
        // Get brake frames
        DPRINT("Enter brakeframes(%f):\r\n", brakeframes);
        uart_get(cmd);
        brakeframes = strtof((char*)cmd, NULL);
        // Get brake 
        DPRINT("Enter brakePWM(%f):\r\n", brakepwm);
        uart_get(cmd);
        brakepwm = strtof((char*)cmd, NULL);
      }
    );
    // Else use buttons...TODO
  }
  
  return 0;
}


void run(float kp, float ki, float kd, float minspeed, float maxspeed, float brakeframes, float brakepwm)
{
  uint16_t line[N_CAM_PNTS];
  uint32_t speed_state,
           wait_cycles;
  struct Result pnts;
  float steer_duty,
        steer_duty_raw,
        midpoint,
        error,
        sharp_steer_dev,
        motor_duty,
        motor_duty_prev,
        diff_steer,
        brake_decrement,
        speed_increment,
        speed_target;

  PID servo_pid;
  
#ifdef DEBUG
  // Input command
  uint8_t cmd[50];
#endif
  
  // 0 speed, 0 deg turn
  SetDCMotDuty(0.0f, 0.0f);
  SetServoDuty(CTR_SERVO_DUTY);
  
#ifdef DUMB_WAY
  pid_init(&servo_pid, 0.0f, kp, ki, kd);
#else
  // Init Servo PID (0.0 is line center)
  pid_init(&servo_pid, 0.0f, kp, ki, kd);
#endif

  // Straight line, straight wheels
  SetDCMotDuty(minspeed, minspeed);
  motor_duty = minspeed;
  steer_duty = CTR_SERVO_DUTY;
  midpoint = N_CAM_PNTS / 2;
  
  speed_state = CONTINUOUS;
  
  // Camera
  for(;;)
  {
    // Get camera line scan output
    get_line(line);
    // Detect line positions
    pnts = find_edges(line, midpoint);
    
    // Off track safety measure
    // TODO: move to state handling logic
    /*if (!pnts.l_pnt && !pnts.r_pnt)
    {
      GPIOB_PCOR = (1 << 22); // Red LED (TODO no debug cam to init RED)
      SetDCMotDuty(minspeed);
      for(uint32_t i=0;i<1000;++i);
      SetDCMotDuty(0);
      return;
    }*/
        
    // Make control adjustments Change steering duty (TODO init servo_pid)
    if (servo_ready())
    {
      // Compute camera midpoint
      midpoint = (float)(pnts.r_pnt + pnts.l_pnt) / 2.0f;
      
#ifdef DUMB_WAY
      // Normalize midpoint to bounds [5,10] for servo
      steer_duty_raw = SERVO_SCALAR * (midpoint - N_CAM_PNTS + 1) * -1.0f + SERVO_BIAS;
      // Double relative error
      steer_duty_raw += (steer_duty_raw - CTR_SERVO_DUTY) * kp;
      steer_duty = steer_duty_raw;
      // Clip value
      CLIP(steer_duty, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
      
      motor_duty_prev = motor_duty;
      
      switch(speed_state)
      {
        case BRAKE:
          motor_duty -= brake_decrement;
          CLIP(motor_duty, MIN_DCMOT_DUTY, maxspeed);
          diff_steer = motor_duty;
          if(motor_duty <= (brakepwm + FLT_EPSILON * 2.0f))
          {
            wait_cycles = brakeframes;
            speed_state = WAIT;
          }
        
        case WAIT:
          if(--wait_cycles == 0)
            speed_state = CONTINUOUS;
        
        case SPEEDUP:
          motor_duty += speed_increment;
          CLIP(motor_duty, minspeed, maxspeed);
          diff_steer = motor_duty;
          if(motor_duty >= (speed_target - FLT_EPSILON * 2.0f))
            speed_state = CONTINUOUS;
          
        case CONTINUOUS:
          // DC Variable Speed
          //motor_duty = maxspeed-(maxspeed-minspeed)*fabsf(CTR_SERVO_DUTY-steer_duty)/(MAX_SERVO_DUTY-CTR_SERVO_DUTY);
          motor_duty = maxspeed / (fabsf(CTR_SERVO_DUTY-steer_duty_raw)*ki);  //ki: how soon to turn hard
          // Clip PWM speed
          CLIP(motor_duty, minspeed, maxspeed);
          
          if((motor_duty_prev - motor_duty) >= (maxspeed - minspeed)*0.75f) // brake when slowing down dangerously
          {
            motor_duty = brakepwm;
            speed_state = BRAKE;
            brake_decrement = (motor_duty_prev - motor_duty) / brakeframes;
            
            motor_duty = motor_duty_prev - brake_decrement;
            diff_steer = motor_duty;
          } else if((motor_duty - motor_duty_prev) >= PWM_DANGER) // carefully speed up if appropriate
          {
            speed_state = SPEEDUP;
            speed_increment = (motor_duty - motor_duty_prev) / SPEEDUP_FRAMES;
            speed_target = motor_duty; // set duty as the target
            
            motor_duty = motor_duty_prev + speed_increment;
            diff_steer = motor_duty;
          } else
          {
            sharp_steer_dev = maxspeed / (minspeed * ki);
            // Differential steering
            if(steer_duty_raw < CTR_SERVO_DUTY - sharp_steer_dev)
            {
              diff_steer = motor_duty / ((CTR_SERVO_DUTY - steer_duty_raw) * kd);
              CLIP(diff_steer, 0, motor_duty);
            } else if(steer_duty_raw > CTR_SERVO_DUTY + sharp_steer_dev)
            {
              diff_steer = motor_duty / ((steer_duty_raw - CTR_SERVO_DUTY) * kd);
              CLIP(diff_steer, 0, motor_duty);
            } else {
              diff_steer = motor_duty;
            }
          }
      }

      SetServoDuty(steer_duty);
#else
      // Normalize midpoint to error bounds [-2.5,+2.5] for servo
      error = NORM_CAM2SERVO(CAM_MID_PNT-midpoint);
      steer_duty += pid_compute(&servo_pid, error);
      CLIP(steer_duty, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
      SetServoDuty(steer_duty);
#endif
      // LED state
      stateSet(steer_duty, motor_duty);

      if(steer_duty < CTR_SERVO_DUTY)
        SetDCMotDuty(diff_steer, motor_duty);
      else
        SetDCMotDuty(motor_duty, diff_steer);
      //SetDCMotDuty(motor_duty, motor_duty);
    }
    
    // Check if command received to stop running
    DBG(
      if(uart_hasdata())
      {
        uart_get(cmd);
        if(!strcmp((char*)cmd, CSTOP))
        {
          SetDCMotDuty(minspeed, minspeed);
          for(uint32_t i=0;i<1000;++i);
          SetDCMotDuty(0.0f, 0.0f);
          return;
        }
      }
    );

    // Debug printing
    DDELAY(750,
      DPRINT("left point: %u, right point: %u\r\n", pnts.l_pnt, pnts.r_pnt);
      DPRINT("midpoint: %f\r\n", midpoint);
      DPRINT("steer_duty: %f\r\n", steer_duty);
      DPRINT("steer_duty_raw: %f\r\n", steer_duty_raw);
      DPRINT("error: %f\r\n", error);
      DPRINT("motor_duty: %f\r\n", motor_duty);
      DPRINT("diff_steer: %f\r\n", diff_steer);
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
  
  // Init LEDs
  LED_Init();
  
#ifdef DEBUG_CAM
  debug_camera();
#endif
}
