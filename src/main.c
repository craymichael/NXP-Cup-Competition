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
 /* file: main.c
  *
  * Responsible for initializing all modules and providing an interface
  * for serial Bluetooth communication to start, stop, and tune runs
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

void initialize(void);
void run(float kp, float ki, float kd, float minspeed, float maxspeed, float brakeframes, float brakepwm, float brakeerror);


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
        brakepwm = BRAKE_PWM,
        brakeerror = BRAKE_ERROR;
  
#ifdef DEBUG
  // Input command
  uint8_t cmd[50];
#endif
  
  // Init K64F modules
  initialize();
  
  DPRINT("Enter Command:\r\n");
  
  for(;;)
  {
    if((GPIOC_PDIR & (1 << 6)) == 0) //SW2
      run(kp, ki, kd, minspeed, maxspeed, brakeframes, brakepwm, brakeerror);
    
    DBG(
      // Wait until BT command received
      if(uart_hasdata())
      {
        uart_get(cmd);
        if(!strcmp((char*)cmd, CSTART))
        {
          // Run with specified paramters
          DPRINT("Running with:\r\n");
          DPRINT("  kp:       %f\r\n", kp);
          DPRINT("  ki:       %f\r\n", ki);
          DPRINT("  kd:       %f\r\n", kd);
          DPRINT("  minspeed: %f\r\n", minspeed);
          DPRINT("  maxspeed: %f\r\n", maxspeed);
          DPRINT("==========================\r\n\r\n");
          run(kp, ki, kd, minspeed, maxspeed, brakeframes, brakepwm, brakeerror);
          DPRINT("Stopping run.\r\n");
        } else if(!strcmp((char*)cmd, CPID))
        {
          // Get KP
          DPRINT("Enter KP(%f):\r\n", kp);
          uart_get(cmd);
          if(strlen((char*)cmd))
            kp = strtof((char*)cmd, NULL);
          // Get KI
          DPRINT("Enter KI(%f):\r\n", ki);
          uart_get(cmd);
          if(strlen((char*)cmd))
            ki = strtof((char*)cmd, NULL);
          // Get KD
          DPRINT("Enter KD(%f):\r\n", kd);
          uart_get(cmd);
          if(strlen((char*)cmd))
            kd = strtof((char*)cmd, NULL);
        } else if(!strcmp((char*)cmd, CSPEEDS))
        {
          // Get minspeed
          DPRINT("Enter minspeed(%f):\r\n", minspeed);
          uart_get(cmd);
          if(strlen((char*)cmd))
            minspeed = strtof((char*)cmd, NULL);
          // Get maxspeed
          DPRINT("Enter maxspeed(%f):\r\n", maxspeed);
          uart_get(cmd);
          if(strlen((char*)cmd))
            maxspeed = strtof((char*)cmd, NULL);
        } else if(!strcmp((char*)cmd, CBRAKE))
        {
          // Get brake frames
          DPRINT("Enter brakeframes(%f):\r\n", brakeframes);
          uart_get(cmd);
          if(strlen((char*)cmd))
            brakeframes = strtof((char*)cmd, NULL);
          // Get brake pwm
          DPRINT("Enter brakePWM(%f):\r\n", brakepwm);
          uart_get(cmd);
          if(strlen((char*)cmd))
            brakepwm = strtof((char*)cmd, NULL);
          // Get brake error
          DPRINT("Enter brakeerror(%f):\r\n", brakeerror);
          uart_get(cmd);
          if(strlen((char*)cmd))
            brakeerror = strtof((char*)cmd, NULL);
        }
        DPRINT("Enter Command:\r\n");
      }
    );
    // Else use buttons...TODO
  }
  
  return 0;
}


void run(float kp, float ki, float kd, float minspeed, float maxspeed, float brakeframes, float brakepwm, float brakeerror)
{
  uint16_t line[N_CAM_PNTS];
  uint32_t speed_state,
           wait_cycles,
           berror;
  struct Result pnts;
  float steer_duty,
        steer_duty_raw,
        midpoint,
        error,
        sharp_steer_dev,
        motor_duty,
        motor_duty_prev,
        diff_steer,
        target_pwm;
  
#ifdef DEBUG
  // Input command
  uint8_t cmd[50];
#endif
  
  // 0 speed, 0 deg turn
  SetDCMotDuty(0.0f, 0.0f);
  SetServoDuty(CTR_SERVO_DUTY);

  // Straight line, straight wheels
  SetDCMotDuty(minspeed, minspeed);
  motor_duty = minspeed;
  steer_duty = CTR_SERVO_DUTY;
  midpoint = N_CAM_PNTS / 2;
  
  speed_state = CONTINUOUS;
  
  berror = 0;
  
  // Camera
  for(;;)
  {
    // Get camera line scan output
    get_line(line);
    // Detect line positions
    pnts = find_edges(line, midpoint);
    /*if (!pnts.l_pnt && !pnts.r_pnt) // Off track safety measure
    {
      SetDCMotDuty(0);
      return;
    }*/
    
    // Compute camera midpoint
    midpoint = (float)(pnts.r_pnt + pnts.l_pnt) / 2.0f;
    
    // Normalize midpoint to bounds [5,10] for servo
    steer_duty_raw = SERVO_SCALAR * (midpoint - N_CAM_PNTS + 1) * -1.0f + SERVO_BIAS;
    // Amplify relative error
    steer_duty_raw += (steer_duty_raw - CTR_SERVO_DUTY) * kp;
    error = CAM_MID_PNT - midpoint; // cam index error
    steer_duty = steer_duty_raw;
    // Clip value
    CLIP(steer_duty, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
    motor_duty_prev = motor_duty;
    
    if(speed_state == CONTINUOUS)
    {
      // DC Variable Speed
      motor_duty = (maxspeed/2.0f) / (fabsf(CTR_SERVO_DUTY-steer_duty_raw)*ki) + maxspeed/2.0f;  //ki: how soon to turn hard
      // Clip PWM speed
      CLIP(motor_duty, minspeed, maxspeed);
      // brake when slowing down dangerously (BRAKE)
      if(fabsf(error) >= brakeerror && berror == 0)
      {
        berror = 1;
        speed_state = BRAKE;
        
        motor_duty = motor_duty_prev - PWM_DANGER;
        diff_steer = motor_duty;
      } // Carefully speed up if appropriate (SPEEDUP)
      else if((motor_duty - motor_duty_prev) >= PWM_DANGER)
      {
        speed_state = SPEEDUP;
        target_pwm = motor_duty; // set duty as the target
        
        motor_duty = motor_duty_prev + PWM_DANGER; // increment by maximum amount permisable
        diff_steer = motor_duty;
      } // CONTINUOUS
      else
      {
        sharp_steer_dev = maxspeed / (minspeed * ki);
        // Differential steering
        if(steer_duty_raw < CTR_SERVO_DUTY - sharp_steer_dev || steer_duty_raw > CTR_SERVO_DUTY + sharp_steer_dev)
        {
          diff_steer = fabsf((motor_duty*2.0f) / ((CTR_SERVO_DUTY - steer_duty_raw) * kd)) - motor_duty;
          CLIP(diff_steer, -motor_duty, motor_duty);
        }
        else
        {
          diff_steer = motor_duty;
        }
      }
    }
    
    // Make control adjustments Change steering duty
    if(servo_ready()) // every ~20ms
    {
      if(fabsf(error) < brakeerror)
        berror = 0;
      
      switch(speed_state)
      {
        case BRAKE:
          // Continue decrementing rear wheel duty until target reached, then wait
          motor_duty = motor_duty_prev - PWM_DANGER;
          CLIP(motor_duty, brakepwm, maxspeed);
          diff_steer = motor_duty;
          if(motor_duty <= (brakepwm + FLT_EPSILON * 2.0f))
          {
            wait_cycles = brakeframes;
            speed_state = WAIT;
          }
          break;
        
        case WAIT:
          // Wait for the specified amount of cycles
          if(--wait_cycles == 0)
            speed_state = CONTINUOUS;
          break;
        
        case SPEEDUP:
          // Speed up gradually until the target is reached
          motor_duty = motor_duty_prev + PWM_DANGER;
          CLIP(motor_duty, motor_duty_prev, target_pwm);
          diff_steer = motor_duty;
          if(motor_duty >= (target_pwm - FLT_EPSILON * 2.0f))
            speed_state = CONTINUOUS;
          break;
      }
      // Update Servo PWM
      SetServoDuty(steer_duty);
      // LED state
      stateSet(steer_duty, motor_duty);
      
      // Debug printing
      DDELAY(200,
        DPRINT("left point: %u, right point: %u\r\n", pnts.l_pnt, pnts.r_pnt);
        DPRINT("==================================\r\n");
        DPRINT("midpoint: %f\r\n", midpoint);
        DPRINT("error: %f\r\n", error);
        DPRINT("==================================\r\n");
        DPRINT("steer_duty: %f\r\n", steer_duty);
        DPRINT("steer_duty_raw: %f\r\n", steer_duty_raw);
        DPRINT("==================================\r\n");
        DPRINT("motor_duty: %f\r\n", motor_duty);
        DPRINT("diff_steer: %f\r\n", diff_steer);
        DPRINT("\r\n");
      );
    }
    
    // Speed Update
    if(steer_duty < CTR_SERVO_DUTY)
      SetDCMotDuty(diff_steer, motor_duty);
    else
      SetDCMotDuty(motor_duty, diff_steer);
    
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
  
  // Buttons
  //initialize clocks for each different port used.
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; 
  // Configure Port Control Register for Inputs with pull enable and pull up resistor
  PORTA_PCR4 = PORT_PCR_PE_MASK |
               PORT_PCR_PS_MASK;
  PORTC_PCR6 |= PORT_PCR_PE_MASK |
                PORT_PCR_PS_MASK;
  PORTA_PCR4 |= PORT_PCR_MUX(1);
  PORTC_PCR6 |= PORT_PCR_MUX(1);
  // Set the push buttons as inputs
  GPIOA_PDDR &= ~(1 << 4); //SW3
  GPIOC_PDDR &= ~(1 << 6); //SW2
  
#ifdef DEBUG_CAM
  debug_camera();
#endif
}
