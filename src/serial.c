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
#include "common.h"
#include <stdio.h>


void uart_init()
{
  // Vars for baud rate and baud rate fine adjust
  uint16_t ubd, brfa;

  // Enable clock for UART module
  SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;

  // Configure the port control register to alternative 3 (which is UART mode for K64)
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable Port B
  PORTB_PCR16 |= PORT_PCR_MUX(0x3);  // PTB16 - UART0_RX
  PORTB_PCR17 |= PORT_PCR_MUX(0x3);  // PTB17 - UART0_TX

  /* Configure the UART for establishing serial communication */
  // Disable transmitter and receiver until proper settings are chosen for the UART module
  UART0_C2 &= ~(UART_C2_RE_MASK |
                UART_C2_TE_MASK);

  // Select default transmission/reception settings for serial communication of UART by clearing the control register 1
  UART0_C1 = 0x0; // 8-bit mode, no parity

  // UART Baud rate is calculated by: baud rate = UART module clock / (16 ?(SBR[12:0] + BRFD))
  // 13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
  // BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
  // BRFA is defined by the lower 4 bits of control register, UART0_C4 
  // Calculate baud rate settings: ubd = UART module clock/16* baud rate
  ubd = (uint16_t)((DEFAULT_SYSTEM_CLOCK)/(BAUD_RATE * 16u));  

  // Clear SBR bits of BDH
  UART0_BDH &= ~UART_BDH_SBR_MASK;

  // Distribute this ubd in BDH and BDL
  UART0_BDH |= UART_BDH_SBR(ubd >> 8u);
  UART0_BDL  = UART_BDL_SBR(ubd);

  // BRFD = (1/32)*BRFA 
  // Make the baud rate closer to the desired value by using BRFA
  brfa = (((DEFAULT_SYSTEM_CLOCK*32u)/(BAUD_RATE * 16u)) - (ubd * 32u));

  // Write the value of brfa in UART0_C4
  UART0_C4 |= UART_C4_BRFA(brfa);

  // Enable transmitter and receiver of UART
  UART0_C2 |= UART_C2_RE_MASK |
              UART_C2_TE_MASK;
}


uint8_t uart_getchar()
{
  /* Wait until there is space for more data in the receiver buffer */
  while(!(UART0_S1 & UART_S1_RDRF_MASK));

  /* Return the 8-bit data from the receiver */
  return UART0_D & UART_D_RT_MASK;
}


void uart_putchar(uint8_t ch)
{
  /* Wait until transmission of previous bit is complete */
  while(!(UART0_S1 & UART_S1_TDRE_MASK));

  /* Send the character */
  UART0_D = ch & UART_D_RT_MASK;
}


void uart_put(uint8_t* ptr_str)
{
  /* Use putchar to print string */
  while(*ptr_str)
    uart_putchar(*ptr_str++);
}


void putnumU(uint32_t i)
{
  /* Put a number using uart_put */
  uint8_t* ptr_str;

  sprintf((char*) ptr_str, "%u", i);
  uart_put(ptr_str);
}


void uart_get(uint8_t* ptr_str)
{
  uint32_t count = 0;

  // Retrieve chars until count surpasses CHAR_COUNT
  while(count < CHAR_COUNT)
  {
    *ptr_str = uart_getchar(); // Get char from UART

    if(*ptr_str == '\r') {  // Terminate str and return if '\r'
      *ptr_str = '\0';
      return;
    }

    uart_putchar(*ptr_str);  // Echo char
    ptr_str++;
    count++;
  }

  // End of loop, terminate str and wait until enter is pressed...
  *ptr_str = '\0';
  while(uart_getchar() != '\r');
}
